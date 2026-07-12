"""Pathfinding for the Fly-in simulation.

Implements Dijkstra (weighted shortest path) and Yen's k-shortest-paths
algorithm from scratch — no graph libraries used.

Zone movement costs:
  normal    -> 1 turn
  priority  -> 1 turn  (but preferred in tie-breaking)
  restricted-> 2 turns
  blocked   -> inaccessible (never entered)
"""

import heapq
from typing import Optional
from models import Graph, Zone, Connection, ZoneType

_INF: float = float('inf')


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
        """Return the number of zones in this path."""
        return len(self.zones)

    def __repr__(self) -> str:
        """Return a human-readable representation of this path."""
        seq = " -> ".join(z.name for z in self.zones)
        return f"Path(cost={self.total_cost}, {seq})"


def _zone_priority_score(zone: Zone) -> int:
    """Return a tiebreak score for a zone (lower = more preferred).

    Args:
        zone: The zone to score.

    Returns:
        int: 0 for PRIORITY zones, 1 for all others.
    """
    return (
        0
        if zone.zone_type == ZoneType.PRIORITY
        else 1
    )


class Dijkstra:
    """Single-source shortest-path finder using Dijkstra's algorithm.

    Finds the lowest-cost path from source to target, optionally
    excluding specific zones or connections (used by Yen's algorithm).

    Example:
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
        """Initialise Dijkstra with a graph, endpoints, and optional bans.

        Args:
            graph: The drone network graph.
            source: Starting zone.
            target: Destination zone.
            banned_zones: Zone names that must not be visited.
            banned_conn_keys: Connection keys that must not be traversed.
        """
        self._graph = graph
        self._source = source
        self._target = target
        self._banned_zones: set[str] = banned_zones or set()
        self._banned_conn_keys: set[str] = banned_conn_keys or set()

    def run(self) -> Optional[Path]:
        """Execute Dijkstra and return the shortest path, or None.

        Returns:
            Optional[Path]: Shortest path from source to target, or None
                            if no path exists.
        """
        dist, tiebreak, prev_zone, prev_conn = self._init_state()
        self._update_distances(dist, tiebreak, prev_zone, prev_conn)

        if dist[self._target.name] == _INF:
            return None
        return self._reconstruct(dist, prev_zone, prev_conn)

    def _init_state(
        self,
    ) -> tuple[
        dict[str, float],
        dict[str, float],
        dict[str, Optional[str]],
        dict[str, Optional[Connection]],
    ]:
        """Initialise distance, tiebreak, and predecessor maps.

        Returns:
            tuple: (dist, tiebreak, prev_zone, prev_conn) maps, with the
                   source node initialised to cost 0.
        """
        names = list(self._graph.zones)
        dist: dict[str, float] = {n: _INF for n in names}
        tiebreak: dict[str, float] = {n: _INF for n in names}
        prev_zone: dict[str, Optional[str]] = {n: None for n in names}
        prev_conn: dict[str, Optional[Connection]] = {
            n: None for n in names
        }
        dist[self._source.name] = 0
        tiebreak[self._source.name] = 0
        return dist, tiebreak, prev_zone, prev_conn

    def _update_distances(
        self,
        dist: dict[str, float],
        tiebreak: dict[str, float],
        prev_zone: dict[str, Optional[str]],
        prev_conn: dict[str, Optional[Connection]],
    ) -> None:
        """Run the main Dijkstra relaxation loop.

        Modifies dist, tiebreak, prev_zone, and prev_conn in place.

        Args:
            dist: Current best cost per zone name.
            tiebreak: Current best tiebreak score per zone name.
            prev_zone: Predecessor zone name per zone name.
            prev_conn: Predecessor connection per zone name.
        """
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
                if (neighbor.name in self._banned_zones):
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
        dist: dict[str, float],
        prev_zone: dict[str, Optional[str]],
        prev_conn: dict[str, Optional[Connection]],
    ) -> Path:
        """Walk predecessor maps back from target to build a Path.

        Args:
            dist: Finalised cost per zone name.
            prev_zone: Predecessor zone name per zone name.
            prev_conn: Predecessor connection per zone name.

        Returns:
            Path: Reconstructed shortest path from source to target.
        """
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
            total_cost=int(dist[self._target.name]),
        )


class YenKShortest:
    """Find up to k distinct shortest paths using Yen's algorithm.

    Iteratively generates candidate paths by deviating from already-found
    paths at each spur node, then selects the cheapest unseen candidate.

    Example:
        paths = YenKShortest(graph, source, target, k=6).run()
    """

    def __init__(
        self,
        graph: Graph,
        source: Zone,
        target: Zone,
        k: int,
    ) -> None:
        """Initialise Yen's k-shortest-paths finder.

        Args:
            graph: The drone network graph.
            source: Starting zone.
            target: Destination zone.
            k: Maximum number of distinct paths to return.
        """
        self._graph = graph
        self._source = source
        self._target = target
        self._k = k

    def run(self) -> list[Path]:
        """Execute Yen's algorithm and return up to k shortest paths.

        Returns:
            list[Path]: Up to k paths ordered by total cost ascending.
                        Empty if no path exists from source to target.
        """
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

            result.append(path)

            ctr = self._generate_spurs(path, result, candidates, ctr)
        return result

    def _generate_spurs(
        self,
        path: Path,
        result: list[Path],
        candidates: list[tuple[int, int, Path]],
        ctr: int,
    ) -> int:
        """Generate spur paths for every spur node along path.

        For each prefix of path, runs Dijkstra with banned connections
        and zones derived from already-found paths, then pushes valid
        candidates onto the heap.

        Args:
            path: The path whose spur nodes to explore.
            result: Already-accepted paths (used to build bans).
            candidates: Min-heap of (cost, ctr, path) pending candidates.
            ctr: Monotonically increasing counter for heap tie-breaking.

        Returns:
            int: Updated counter value after all insertions.
        """
        for i in range(len(path.zones) - 1):
            spur_node = path.zones[i]
            root = path.zones[: i + 1]

            banned_c, banned_z = self._build_bans(i, root, result)

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
    ) -> tuple[set[str], set[str]]:
        """Build banned connections and zones for a spur search.

        Bans the outgoing connection from the spur node for every
        already-accepted path that shares the same root prefix, and
        bans all interior root zones to prevent cycles.

        Args:
            i: Index of the spur node in path.zones.
            root: Prefix of path up to and including the spur node.
            result: Already-accepted paths.

        Returns:
            tuple[set[str], set[str]]: (banned_conn_keys, banned_zone_names).
        """
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
        """Concatenate a root prefix with a spur path into a full path.

        Args:
            path: Original path providing the root connections prefix.
            spur: Spur path from the spur node to the target.
            root: Zone prefix up to and including the spur node.
            i: Index of the spur node, used to slice path.connections.

        Returns:
            Path: Combined candidate path with recalculated total cost.
        """
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


def dijkstra(
    graph: Graph,
    source: Zone,
    target: Zone,
    banned_zones: Optional[set[str]] = None,
    banned_conn_keys: Optional[set[str]] = None,
) -> Optional[Path]:
    """Find the shortest path from source to target (Dijkstra wrapper).

    Args:
        graph: The drone network graph.
        source: Starting zone.
        target: Destination zone.
        banned_zones: Zone names to exclude from the search.
        banned_conn_keys: Connection keys to exclude from the search.

    Returns:
        Optional[Path]: Shortest path, or None if unreachable.
    """
    return Dijkstra(
        graph, source, target, banned_zones, banned_conn_keys
    ).run()


def find_k_shortest_paths(
    graph: Graph,
    source: Zone,
    target: Zone,
    k: int,
) -> list[Path]:
    """Find up to k shortest paths from source to target (Yen wrapper).

    Args:
        graph: The drone network graph.
        source: Starting zone.
        target: Destination zone.
        k: Maximum number of paths to return.

    Returns:
        list[Path]: Up to k paths ordered by total cost ascending.
    """
    return YenKShortest(graph, source, target, k).run()
