"""Parser for Fly-in drone network map files.

Implements all constraints from section VII.4 of the subject.
Any parse error stops the program with a clear message indicating
line number and cause.
"""

import re
from typing import Optional
from models import Graph, Zone, ZoneType, Connection


class ParseError(Exception):
    """Raised when the map file contains a syntax or semantic error."""

    def __init__(self, line_number: int, message: str) -> None:
        super().__init__(f"Line {line_number}: {message}")
        self.line_number = line_number
        self.message = message


class MapParser:
    """Parses a drone network map file into a Graph.

    Example:
        graph = MapParser("map.txt").parse()
    """

    def __init__(self, filepath: str) -> None:
        self.filepath = filepath
        self._graph = Graph()
        self._seen_connections: set[str] = set()
        self._has_start = False
        self._has_end = False
        self._nb_drones_set = False

    def parse(self) -> Graph:
        """Parse the map file and return a populated Graph.

        Returns:
            Graph: Fully parsed graph with all zones and connections.

        Raises:
            FileNotFoundError: If filepath does not exist.
            ParseError: On any syntax or semantic error in the map file.
        """
        lines = self._read_file()
        for line_num, raw_line in enumerate(lines, start=1):
            self._process_line(raw_line, line_num)
        self._validate_completeness()
        return self._graph

    def _read_file(self) -> list[str]:
        """Read all lines from the map file.

        Returns:
            list[str]: Raw lines including newline characters.

        Raises:
            FileNotFoundError: If the file does not exist.
        """
        try:
            with open(self.filepath, "r") as f:
                return f.readlines()
        except FileNotFoundError:
            raise FileNotFoundError(
                f"Map file not found: '{self.filepath}'"
            )
        except PermissionError:
            raise PermissionError(
                f"Permission denied when reading map file: '{self.filepath}'"
            )
        except OSError as exc:
            raise OSError(
                f"Could not read map file '{self.filepath}': {exc}"
            )

    def _process_line(self, raw_line: str, line_num: int) -> None:
        """Strip and dispatch a single raw line to the appropriate handler.

        Blank lines and lines starting with '#' are silently skipped.

        Raises:
            ParseError: If the line is unrecognised or violates ordering rules.
        """
        line = raw_line.strip()

        if not line or line.startswith("#"):
            return

        if "#" in line:
            for i, c in enumerate(line):
                if c == "#":
                    line = line[:i]
                    break

        if not self._nb_drones_set and not line.startswith("nb_drones:"):
            raise ParseError(
                line_num,
                f"The first non-comment line must be 'nb_drones: "
                f"<positive_integer>', got: '{line}'"
            )

        if line.startswith("nb_drones:"):
            self._handle_nb_drones(line, line_num)
        elif line.startswith("start_hub:"):
            self._handle_start_hub(line, line_num)
        elif line.startswith("end_hub:"):
            self._handle_end_hub(line, line_num)
        elif line.startswith("hub:"):
            self._handle_hub(line, line_num)
        elif line.startswith("connection:"):
            self._handle_connection(line, line_num)
        else:
            raise ParseError(line_num, f"Unrecognized line: '{line}'")

    def _handle_nb_drones(self, line: str, line_num: int) -> None:
        """Parse and store the nb_drones directive.

        Raises:
            ParseError: On duplicate definition or non-positive integer.
        """
        if self._nb_drones_set:
            raise ParseError(line_num, "Duplicate nb_drones definition")
        raw = line[len("nb_drones:"):].strip()
        try:
            nb = int(raw)
            if nb <= 0:
                raise ValueError()
            if 6600 < nb:
                raise ParseError(
                    line_num,
                    f"nb_drones more then 6600, got '{nb}'"
                )
        except ValueError:
            raise ParseError(
                line_num,
                f"nb_drones must be a positive integer, got '{raw}'"
            )
        self._graph.nb_drones = nb
        self._nb_drones_set = True

    def _handle_start_hub(self, line: str, line_num: int) -> None:
        """Parse and register the start hub zone.

        Raises:
            ParseError: On duplicate start_hub or invalid zone syntax.
        """
        if self._has_start:
            raise ParseError(line_num, "Duplicate start_hub definition")
        rest = line[len("start_hub:"):].strip()
        zone = self._parse_zone_line(
            rest, line_num, is_start=True, is_end=False
        )
        self._add_zone(zone, line_num)
        self._has_start = True

    def _handle_end_hub(self, line: str, line_num: int) -> None:
        """Parse and register the end hub zone.

        Raises:
            ParseError: On duplicate end_hub or invalid zone syntax.
        """
        if self._has_end:
            raise ParseError(line_num, "Duplicate end_hub definition")
        rest = line[len("end_hub:"):].strip()
        zone = self._parse_zone_line(
            rest, line_num, is_start=False, is_end=True
        )
        self._add_zone(zone, line_num)
        self._has_end = True

    def _handle_hub(self, line: str, line_num: int) -> None:
        """Parse and register an intermediate hub zone.

        Raises:
            ParseError: On duplicate zone name or invalid zone syntax.
        """
        rest = line[len("hub:"):].strip()
        zone = self._parse_zone_line(
            rest, line_num, is_start=False, is_end=False
        )
        self._add_zone(zone, line_num)

    def _handle_connection(self, line: str, line_num: int) -> None:
        """Parse and register a connection between two zones.

        Raises:
            ParseError: On duplicate connection or invalid connection syntax.
        """
        rest = line[len("connection:"):].strip()
        conn = self._parse_connection_line(rest, line_num)
        key = conn.key()
        if key in self._seen_connections:
            raise ParseError(
                line_num,
                f"Duplicate connection '{key}' "
                f"(a-b and b-a are considered the same)"
            )
        self._seen_connections.add(key)
        self._graph.add_connection(conn)

    def _parse_zone_line(
        self,
        rest: str,
        line_num: int,
        is_start: bool,
        is_end: bool,
    ) -> Zone:
        """Parse zone attributes from the portion after the directive prefix.

        Raises:
            ParseError: On missing fields, bad coordinates, invalid zone
                        type, non-positive max_drones, or unknown metadata.
        """
        rest, meta = self._extract_metadata(
            rest, line_num, allow_bracket_names=False
        )
        parts = rest.split()
        if len(parts) != 3:
            raise ParseError(
                line_num,
                f"Expected '<name> <x> <y>' but got '{rest}'"
            )

        name = parts[0]
        if "-" in name:
            raise ParseError(
                line_num, f"Zone name '{name}' must not contain dashes"
            )

        try:
            x, y = int(parts[1]), int(parts[2])
        except ValueError:
            raise ParseError(
                line_num,
                f"Zone coordinates must be integers, "
                f"got '{parts[1]}' and '{parts[2]}'"
            )

        zone_type_str = meta.get("zone", "normal")
        try:
            zone_type = ZoneType(zone_type_str)
        except ValueError:
            raise ParseError(
                line_num,
                f"Invalid zone type '{zone_type_str}'. "
                f"Allowed: normal, blocked, restricted, priority"
            )

        color: Optional[str] = meta.get("color", None)
        if is_start or is_end:
            max_drones_raw = str(self._graph.nb_drones)

        else:
            max_drones_raw = meta.get("max_drones", "1")

        try:
            max_drones = int(max_drones_raw)
            if max_drones <= 0:
                raise ValueError()
        except ValueError:
            raise ParseError(
                line_num,
                f"max_drones must be a positive integer, "
                f"got '{max_drones_raw}'"
            )

        known = {"zone", "color", "max_drones"}
        for k in meta:
            if k not in known:
                raise ParseError(
                    line_num, f"Unknown zone metadata key '{k}'"
                )

        return Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            color=color,
            max_drones=max_drones,
            is_start=is_start,
            is_end=is_end,
        )

    def _parse_connection_line(self, rest: str, line_num: int) -> Connection:
        """Parse connection attributes from the portion after 'connection:'.

        Raises:
            ParseError: On bad format, unknown zones, self-loop, non-positive
                        max_link_capacity, or unknown metadata keys.
        """
        rest, meta = self._extract_metadata(
            rest, line_num, allow_bracket_names=True
        )
        parts = rest.strip().split()
        if len(parts) != 1:
            raise ParseError(
                line_num,
                f"Connection spec must be exactly '<zone1>-<zone2>', "
                f"got '{rest}'"
            )

        spec = parts[0]

        if spec.count("-") != 1:
            raise ParseError(
                line_num,
                f"Connection '{spec}' must contain exactly "
                f"one dash separating two zone names"
            )

        name_a, name_b = spec.split("-")

        if not name_a or not name_b:
            raise ParseError(
                line_num, f"Empty zone name in connection '{spec}'"
            )

        zone_a = self._graph.get_zone(name_a)
        if zone_a is None:
            raise ParseError(
                line_num,
                f"Unknown zone '{name_a}' referenced in connection"
            )
        zone_b = self._graph.get_zone(name_b)
        if zone_b is None:
            raise ParseError(
                line_num,
                f"Unknown zone '{name_b}' referenced in connection"
            )
        if zone_a is zone_b:
            raise ParseError(
                line_num,
                f"A connection cannot link a zone to itself: '{name_a}'"
            )

        cap_raw = meta.get("max_link_capacity", "1")
        try:
            cap = int(cap_raw)
            if cap <= 0:
                raise ValueError()
        except ValueError:
            raise ParseError(
                line_num,
                f"max_link_capacity must be a positive integer, "
                f"got '{cap_raw}'"
            )

        known = {"max_link_capacity"}
        for k in meta:
            if k not in known:
                raise ParseError(
                    line_num, f"Unknown connection metadata key '{k}'"
                )

        return Connection(zone_a=zone_a, zone_b=zone_b, max_link_capacity=cap)

    def _extract_metadata(
        self, rest: str, line_num: int, allow_bracket_names: bool = False
    ) -> tuple[str, dict[str, str]]:
        """Extract a single trailing metadata block from a line.

        A trailing '[...]' block is treated as metadata only if its contents
        contain '='. Otherwise it is either silently kept as part of the
        content (allow_bracket_names=True, used for connections) or rejected
        as an error (allow_bracket_names=False, used for zone lines).

        Raises:
            ParseError: On missing whitespace separator, duplicate
                        blocks, empty block, or invalid trailing
                        bracket in zone context.
        """
        rest = rest.strip()
        match = re.search(r"\[[^\[\]]*\]$", rest)
        if not match:
            return rest, {}

        block = match.group()
        raw_meta = block[1:-1]

        if "=" not in raw_meta:
            if allow_bracket_names:
                return rest, {}
            raise ParseError(
                line_num,
                f"Invalid trailing block '{block}' — "
                f"did you mean to add metadata like [key=value]?"
            )

        before = rest[:match.start()]

        if before and not before[-1].isspace():
            raise ParseError(
                line_num, "Metadata block must be separated by whitespace"
            )

        before = before.rstrip()

        if re.search(r"\s\[[^\[\]]*\]", before):
            raise ParseError(
                line_num, "Duplicate metadata blocks are not allowed"
            )

        return before, self._parse_metadata(raw_meta, line_num)

    @staticmethod
    def _parse_metadata(raw: str, line_num: int) -> dict[str, str]:
        """Parse key=value pairs from a metadata block's inner content.

        Raises:
            ParseError: On tokens missing '=', or empty key or value.
        """
        meta: dict[str, str] = {}
        for token in raw.strip().split():
            if "=" not in token:
                raise ParseError(
                    line_num,
                    f"Invalid metadata token '{token}' — expected key=value"
                )
            key, _, value = token.partition("=")
            if not key or not value:
                raise ParseError(
                    line_num,
                    f"Empty key or value in metadata token '{token}'"
                )
            key = key.strip()
            value = value.strip()
            if key in meta:
                raise ParseError(
                    line_num,
                    f"Duplicate metadata key '{key}' in block"
                )
            meta[key] = value
        return meta

    def _add_zone(self, zone: Zone, line_num: int) -> None:
        """Add a zone to the graph, raising on duplicate names.

        Raises:
            ParseError: If a zone with the same name already exists.
        """
        if self._graph.get_zone(zone.name) is not None:
            raise ParseError(
                line_num, f"Duplicate zone name '{zone.name}'"
            )
        self._graph.add_zone(zone)

    def _validate_completeness(self) -> None:
        """Verify that all required directives were present in the file.

        Raises:
            ParseError: If nb_drones, start_hub, or end_hub is missing.
        """
        if not self._nb_drones_set:
            raise ParseError(0, "Missing required 'nb_drones:' definition")
        if not self._has_start:
            raise ParseError(0, "Missing required 'start_hub:' definition")
        if not self._has_end:
            raise ParseError(0, "Missing required 'end_hub:' definition")


def parse_map(filepath: str) -> Graph:
    """Parse a map file and return its Graph (backward-compatible wrapper).

    Raises:
        FileNotFoundError: If the file does not exist.
        ParseError: On any syntax or semantic error.
    """
    return MapParser(filepath).parse()
