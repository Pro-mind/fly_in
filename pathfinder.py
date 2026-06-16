"""Pathfinding for the Fly-in simulation.

Implements Dijkstra (weighted shortest path) and Yen's k-shortest-paths
algorithm from scratch — no graph libraries used.

Zone movement costs:
  normal    -> 1 turn
  priority  -> 1 turn  (but preferred in tie-breaking)
  restricted-> 2 turns
  blocked   -> inaccessible (never entered)
"""

from __future__ import annotations
import heapq
from typing import Optional
from models import Graph, Zone, Connection, ZoneType


class Path:
    """A complete route from start to end with cost metadata."""

    def __init__(
        self,
        zones: list[Zone],
        connections: list[Connection],
        total_cost: int,
    ) -> None:
        """Initialize a Path.

        Args:
            zones: Ordered zone list, zones[0]=start, zones[-1]=end.
            connections: connections[i] links zones[i] to zones[i+1].
            total_cost: Total movement cost (sum of destination zone costs).
        """
        self.zones = zones
        self.connections = connections
        self.total_cost = total_cost

    def zone_names(self) -> list[str]:
        """Return ordered list of zone names for this path."""
        return [z.name for z in self.zones]

    def __len__(self) -> int:
        return len(self.zones)

    def __repr__(self) -> str:
        seq = " -> ".join(z.name for z in self.zones)
        return f"Path(cost={self.total_cost}, {seq})"


# ---------------------------------------------------------------------------
# Priority tiebreak: lower = better. PRIORITY zones score 0, others score 1.
# ---------------------------------------------------------------------------
_PRIORITY_BONUS = 0
_NORMAL_PENALTY = 1


def _zone_priority_score(zone: Zone) -> int:
    """Return a tiebreak score for a zone (lower = more preferred)."""
    return (
        _PRIORITY_BONUS
        if zone.zone_type == ZoneType.PRIORITY
        else _NORMAL_PENALTY
    )


# ---------------------------------------------------------------------------
# Dijkstra
# ---------------------------------------------------------------------------

class Dijkstra:
    """Single-source shortest-path finder using Dijkstra's algorithm.

    Usage:
        path = Dijkstra(graph, source, target).run()
        path = Dijkstra(graph, source, target,
                        banned_zones={"X"},
                        banned_conn_keys={"A-B"}).run()
    """

    def __init__(
        self,
        graph: Graph,
        source: Zone,
        target: Zone,
        banned_zones: Optional[set[str]] = None,
        banned_conn_keys: Optional[set[str]] = None,
    ) -> None:
        self._graph = graph
        self._source = source
        self._target = target
        self._banned_zones: set[str] = banned_zones or set()
        self._banned_conn_keys: set[str] = banned_conn_keys or set()

    def run(self) -> Optional[Path]:
        """Execute Dijkstra and return the shortest path, or None."""
        dist, tiebreak, prev_zone, prev_conn = self._init_state()
        self._update_distances(dist, tiebreak, prev_zone, prev_conn)

        if dist[self._target.name] == 10 ** 9:
            return None
        return self._reconstruct(dist, prev_zone, prev_conn)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init_state(
        self,
    ) -> tuple[
        dict[str, int],
        dict[str, int],
        dict[str, Optional[str]],
        dict[str, Optional[Connection]],
    ]:
        INF = 10 ** 9
        names = list(self._graph.zones)
        dist: dict[str, int] = {n: INF for n in names}
        tiebreak: dict[str, int] = {n: INF for n in names}
        prev_zone: dict[str, Optional[str]] = {n: None for n in names}
        prev_conn: dict[str, Optional[Connection]] = {
            n: None for n in names
        }
        dist[self._source.name] = 0
        tiebreak[self._source.name] = 0
        return dist, tiebreak, prev_zone, prev_conn

    def _update_distances(
        self,
        dist: dict[str, int],
        tiebreak: dict[str, int],
        prev_zone: dict[str, Optional[str]],
        prev_conn: dict[str, Optional[Connection]],
    ) -> None:
        counter = 0
        heap: list[tuple[int, int, int, str]] = [
            (0, 0, counter, self._source.name)
        ]
        while heap:
            cost, tb, _, name = heapq.heappop(heap)
            if cost > dist[name]:
                continue
            if cost == dist[name] and tb > tiebreak[name]:
                continue
            if name == self._target.name:
                break

            zone = self._graph.zones[name]
            for neighbor, conn in self._graph.neighbors(zone):
                if (
                    neighbor.name in self._banned_zones
                    and not neighbor.is_end
                ):
                    continue
                if conn.key() in self._banned_conn_keys:
                    continue

                new_cost = cost + neighbor.movement_cost()
                new_tb = tb + _zone_priority_score(neighbor)

                better = new_cost < dist[neighbor.name]
                same_cost_better_tb = (
                    new_cost == dist[neighbor.name]
                    and new_tb < tiebreak[neighbor.name]
                )
                if better or same_cost_better_tb:
                    dist[neighbor.name] = new_cost
                    tiebreak[neighbor.name] = new_tb
                    prev_zone[neighbor.name] = name
                    prev_conn[neighbor.name] = conn
                    counter += 1
                    heapq.heappush(
                        heap,
                        (new_cost, new_tb, counter, neighbor.name)
                    )

    def _reconstruct(
        self,
        dist: dict[str, int],
        prev_zone: dict[str, Optional[str]],
        prev_conn: dict[str, Optional[Connection]],
    ) -> Path:
        zone_list: list[Zone] = []
        conn_list: list[Connection] = []
        cur: Optional[str] = self._target.name

        while cur is not None:
            zone_list.append(self._graph.zones[cur])
            c = prev_conn[cur]
            if c is not None:
                conn_list.append(c)
            cur = prev_zone[cur]

        zone_list.reverse()
        conn_list.reverse()
        return Path(
            zones=zone_list,
            connections=conn_list,
            total_cost=dist[self._target.name],
        )


# ---------------------------------------------------------------------------
# Yen's k-shortest paths
# ---------------------------------------------------------------------------

class YenKShortest:
    """Find up to k distinct shortest paths using Yen's algorithm.

    Usage:
        paths = YenKShortest(graph, source, target, k=6).run()
    """

    def __init__(
        self,
        graph: Graph,
        source: Zone,
        target: Zone,
        k: int = 6,
    ) -> None:
        self._graph = graph
        self._source = source
        self._target = target
        self._k = k

    def run(self) -> list[Path]:
        """Execute Yen's algorithm and return up to k shortest paths."""
        first = Dijkstra(
            self._graph, self._source, self._target
        ).run()
        if first is None:
            return []

        result: list[Path] = []
        candidates: list[tuple[int, int, Path]] = []
        ctr = 0
        heapq.heappush(candidates, (first.total_cost, ctr, first))
        ctr += 1

        while candidates and len(result) < self._k:
            _, _, path = heapq.heappop(candidates)

            if any(p.zone_names() == path.zone_names() for p in result):
                continue
            result.append(path)

            ctr = self._generate_spurs(path, result, candidates, ctr)

        return result

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _generate_spurs(
        self,
        path: Path,
        result: list[Path],
        candidates: list[tuple[int, int, Path]],
        ctr: int,
    ) -> int:
        for i in range(len(path.zones) - 1):
            spur_node = path.zones[i]
            root = path.zones[: i + 1]

            banned_c, banned_z = self._build_bans(i, root, result, path)

            spur = Dijkstra(
                self._graph,
                spur_node,
                self._target,
                banned_zones=banned_z,
                banned_conn_keys=banned_c,
            ).run()
            if spur is None:
                continue

            candidate = self._combine(path, spur, root, i)

            already = any(
                candidate.zone_names() == p.zone_names()
                for _, _, p in candidates
            )
            if not already:
                heapq.heappush(
                    candidates,
                    (candidate.total_cost, ctr, candidate)
                )
                ctr += 1
        return ctr

    @staticmethod
    def _build_bans(
        i: int,
        root: list[Zone],
        result: list[Path],
        path: Path,
    ) -> tuple[set[str], set[str]]:
        banned_c: set[str] = set()
        banned_z: set[str] = set()

        for existing in result:
            if (
                len(existing.zones) > i
                and existing.zones[: i + 1] == root
                and i < len(existing.connections)
            ):
                banned_c.add(existing.connections[i].key())

        for rz in root[:-1]:
            banned_z.add(rz.name)

        return banned_c, banned_z

    @staticmethod
    def _combine(
        path: Path,
        spur: Path,
        root: list[Zone],
        i: int,
    ) -> Path:
        full_zones = root[:-1] + spur.zones
        full_conns: list[Connection] = (
            list(path.connections[:i]) + list(spur.connections)
        )
        total = sum(z.movement_cost() for z in full_zones[1:])
        return Path(
            zones=full_zones,
            connections=full_conns,
            total_cost=total,
        )


# ---------------------------------------------------------------------------
# Convenience wrappers (backward-compatible)
# ---------------------------------------------------------------------------

def dijkstra(
    graph: Graph,
    source: Zone,
    target: Zone,
    banned_zones: Optional[set[str]] = None,
    banned_conn_keys: Optional[set[str]] = None,
) -> Optional[Path]:
    """Backward-compatible wrapper around Dijkstra."""
    return Dijkstra(
        graph, source, target, banned_zones, banned_conn_keys
    ).run()


def find_k_shortest_paths(
    graph: Graph,
    source: Zone,
    target: Zone,
    k: int = 6,
) -> list[Path]:
    """Backward-compatible wrapper around YenKShortest."""
    return YenKShortest(graph, source, target, k).run()
