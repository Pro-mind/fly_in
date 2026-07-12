"""Core data models for the Fly-in drone routing simulation."""

from enum import Enum
from typing import Optional


class ZoneType(Enum):
    """Zone type enumeration with movement costs."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    def movement_cost(self) -> int:
        """Return turns required to enter this zone type.

        Returns:
            int: 2 for restricted, 1 for all others.
        """
        if self == ZoneType.RESTRICTED:
            return 2
        return 1

    def is_accessible(self) -> bool:
        """Return whether drones can enter this zone.

        Returns:
            bool: False only for BLOCKED zones.
        """
        return self != ZoneType.BLOCKED


class Zone:
    """Represents a node/zone in the drone network graph."""

    def __init__(
        self,
        name: str,
        x: int,
        y: int,
        zone_type: ZoneType = ZoneType.NORMAL,
        color: Optional[str] = None,
        max_drones: int = 1,
        is_start: bool = False,
        is_end: bool = False,
    ) -> None:
        """Initialize a Zone.

        Args:
            name: Unique zone identifier (no dashes or spaces).
            x: X coordinate (integer).
            y: Y coordinate (integer).
            zone_type: Type affecting movement cost and accessibility.
            color: Optional display color name.
            max_drones: Maximum simultaneous drones allowed (default 1).
            is_start: True if this is the start hub.
            is_end: True if this is the end hub.
        """
        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.color = color
        self.max_drones = max_drones
        self.is_start = is_start
        self.is_end = is_end

    def movement_cost(self) -> int:
        """Return the movement cost to enter this zone.

        Returns:
            int: Number of turns required.
        """
        return self.zone_type.movement_cost()

    def is_accessible(self) -> bool:
        """Check whether drones may enter this zone.

        Returns:
            bool: True unless zone is BLOCKED.
        """
        return self.zone_type.is_accessible()

    def __repr__(self) -> str:
        """Return debug string representation.

        Returns:
            str: Zone name and type.
        """
        return f"Zone({self.name}, {self.zone_type.value})"


class Connection:
    """Represents a bidirectional edge between two zones."""

    def __init__(
        self,
        zone_a: Zone,
        zone_b: Zone,
        max_link_capacity: int = 1,
    ) -> None:
        """Initialize a Connection.

        Args:
            zone_a: First zone endpoint.
            zone_b: Second zone endpoint.
            max_link_capacity: Max drones traversing simultaneously.
        """
        self.zone_a = zone_a
        self.zone_b = zone_b
        self.max_link_capacity = max_link_capacity

    def other(self, zone: Zone) -> Zone:
        """Return the zone at the other end of this connection.

        Args:
            zone: One endpoint zone.

        Returns:
            Zone: The opposite endpoint.
        """
        if zone is self.zone_a:
            return self.zone_b
        return self.zone_a

    def connects(self, a: Zone, b: Zone) -> bool:
        """Check if this connection links exactly zones a and b.

        Args:
            a: First zone.
            b: Second zone.

        Returns:
            bool: True if this connection links a and b in either direction.
        """
        return (
            (self.zone_a is a and self.zone_b is b)
            or (self.zone_a is b and self.zone_b is a)
        )

    def key(self) -> str:
        """Return a canonical sorted string key for duplicate detection.

        Returns:
            str: Alphabetically sorted 'nameA-nameB'.
        """
        names = sorted([self.zone_a.name, self.zone_b.name])
        return f"{names[0]}-{names[1]}"

    def transit_label(self, from_zone: Zone) -> str:
        """Return the connection label for restricted-zone transit output.

        Per the spec, a drone mid-transit to a restricted zone outputs
        D<ID>-<connection> where <connection> is 'fromZone-toZone'.

        Args:
            from_zone: The zone the drone departed from.

        Returns:
            str: e.g. 'hub-roof1'.
        """
        dest = self.other(from_zone)
        return f"{from_zone.name}-{dest.name}"

    def __repr__(self) -> str:
        """Return debug string representation.

        Returns:
            str: Connection endpoints and capacity.
        """
        return (
            f"Connection({self.zone_a.name}-{self.zone_b.name},"
            f" cap={self.max_link_capacity})"
        )


class DroneStatus(Enum):
    """Lifecycle status of a drone in the simulation."""

    WAITING = "waiting"
    MOVING = "moving"
    IN_TRANSIT = "in_transit"
    DELIVERED = "delivered"


class Drone:
    """Represents a single drone in the simulation."""

    def __init__(self, drone_id: int, start_zone: Zone) -> None:
        """Initialize a Drone at its start zone.

        Args:
            drone_id: Unique positive integer identifier.
            start_zone: The zone all drones begin in.
        """
        self.drone_id = drone_id
        self.current_zone: Zone = start_zone
        self.status: DroneStatus = DroneStatus.WAITING
        self.path_zones: list[Zone] = []
        self.path_connections: list[Connection] = []
        self.path_index: int = 1
        self.departure_turn: int = 0
        self.transit_conn: Optional[Connection] = None
        self.transit_dest: Optional[Zone] = None

    @property
    def label(self) -> str:
        """Drone display label e.g. 'D1'.

        Returns:
            str: Drone identifier string.
        """
        return f"D{self.drone_id}"

    def is_delivered(self) -> bool:
        """Whether the drone has reached the end zone.

        Returns:
            bool: True if DELIVERED.
        """
        return self.status == DroneStatus.DELIVERED

    def is_in_transit(self) -> bool:
        """Whether drone is mid-restricted-zone crossing.

        Returns:
            bool: True if IN_TRANSIT.
        """
        return self.status == DroneStatus.IN_TRANSIT

    def next_zone(self) -> Optional[Zone]:
        """Return the next zone in this drone's path.

        Returns:
            Optional[Zone]: Next destination zone, or None if finished.
        """
        if self.path_index >= len(self.path_zones):
            return None
        return self.path_zones[self.path_index]

    def next_conn(self) -> Connection:
        """Return the connection leading to the next zone in the path.

        connections[i] leads from path_zones[i] to path_zones[i+1],
        so the connection to path_zones[path_index] is at index
        path_index - 1.

        Returns:
            Optional[Connection]: Connection to traverse, or None.
        """
        idx = self.path_index - 1
        return self.path_connections[idx]

    def advance(self) -> None:
        """Advance path index: the drone has entered its next zone."""
        self.path_index += 1

    def __repr__(self) -> str:
        """Return debug string representation.

        Returns:
            str: Drone label and current zone.
        """
        return f"Drone({self.label}, at={self.current_zone.name})"


class Graph:
    """The complete drone network as an adjacency graph."""

    def __init__(self) -> None:
        """Initialize an empty Graph."""
        self.zones: dict[str, Zone] = {}
        self.connections: list[Connection] = []
        self._adj: dict[str, list[Connection]] = {}
        self.start_zone: Optional[Zone] = None
        self.end_zone: Optional[Zone] = None
        self.nb_drones: int = 0

    def add_zone(self, zone: Zone) -> None:
        """Register a zone in the graph.

        Args:
            zone: Zone to add.
        """
        self.zones[zone.name] = zone
        self._adj[zone.name] = []
        if zone.is_start:
            self.start_zone = zone
        if zone.is_end:
            self.end_zone = zone

    def add_connection(self, conn: Connection) -> None:
        """Register a bidirectional connection in the graph.

        Args:
            conn: Connection to add.
        """
        self.connections.append(conn)
        self._adj[conn.zone_a.name].append(conn)
        self._adj[conn.zone_b.name].append(conn)

    def get_zone(self, name: str) -> Optional[Zone]:
        """Look up a zone by name.

        Args:
            name: Zone name.

        Returns:
            Optional[Zone]: Zone or None if not found.
        """
        return self.zones.get(name)

    def neighbors(
        self, zone: Zone
    ) -> list[tuple[Zone, Connection]]:
        """Return all accessible neighbors with their connecting edges.

        Args:
            zone: The zone to query.

        Returns:
            list[tuple[Zone, Connection]]: (neighbor, connection) pairs,
                excluding BLOCKED zones.
        """
        result: list[tuple[Zone, Connection]] = []
        for conn in self._adj.get(zone.name, []):
            neighbor = conn.other(zone)
            if neighbor.is_accessible():
                result.append((neighbor, conn))
        return result

    def get_connection(
        self, a: Zone, b: Zone
    ) -> Optional[Connection]:
        """Find the connection between two specific zones.

        Args:
            a: First zone.
            b: Second zone.

        Returns:
            Optional[Connection]: The connection or None if not adjacent.
        """
        for conn in self._adj.get(a.name, []):
            if conn.connects(a, b):
                return conn
        return None
