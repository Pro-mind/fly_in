"""Simulation engine for the Fly-in drone routing system.

Implements all rules from Chapter VII of the subject:
  - Zone occupancy limits (max_drones, special rules for start/end)
  - Connection capacity limits (max_link_capacity)
  - Movement cost: normal=1 turn, restricted=2 turns, priority=1 turn
  - Restricted-zone transit: drone commits to link on turn T,
    MUST arrive at destination on turn T+1 (cannot wait on the link)
  - Simultaneous movement: drones vacating a zone free capacity for
    incoming drones on the SAME turn
  - Output format: D<ID>-<zone> or D<ID>-<conn_label> for in-transit
"""

from typing import Optional
from models import Graph, Zone, Connection, Drone, DroneStatus
from pathfinder import Path, find_k_shortest_paths, dijkstra


class TurnAction:
    """Records what a single drone does during one simulation turn."""

    def __init__(
        self,
        drone: Drone,
        moved_to: Zone,
        conn_label: str = "",
    ) -> None:
        """Initialize a TurnAction.

        Args:
            drone: The drone that acted.
            moved_to: The zone the drone arrived at (or is heading to).
            conn_label: Non-empty only for restricted-zone transit (turn 1).
                        Contains 'fromZone-toZone' connection label.
        """
        self.drone = drone
        self.moved_to = moved_to
        self.conn_label = conn_label

    def output_token(self) -> str:
        """Return the official output token for this action.

        For normal moves: 'D1-zoneName'
        For restricted transit (in-flight): 'D1-fromZone-toZone'
        """
        if self.conn_label:
            return f"{self.drone.label}-{self.conn_label}"
        return f"{self.drone.label}-{self.moved_to.name}"

    def __repr__(self) -> str:
        return f"TurnAction({self.output_token()})"


class RoutePlanner:
    """Assigns an optimal path and departure turn to every drone.

    Strategy — maximise throughput on optimal-cost paths:
      1. Find the single shortest path cost (Dijkstra).
      2. Collect ALL paths that share that minimum cost (optimal set).
      3. Assign drones exclusively to optimal paths, staggering departures
         to prevent zone/link conflicts.
      4. Distribute drones across optimal paths by capacity-weighted
         round-robin so high-throughput paths absorb more drones.

    Usage:
        RoutePlanner(graph, drones).plan()
    """

    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        assert graph.start_zone is not None
        assert graph.end_zone is not None
        self._graph = graph
        self._drones = drones

    def plan(self) -> None:
        """Assign paths and departure turns to all drones (in place)."""
        shortest_paths = self._find_optimal_paths()
        capacities = [self._path_capacity(p) for p in shortest_paths]
        self._assign(shortest_paths, capacities)

    def _find_optimal_paths(self) -> list[Path]:
        start = self._graph.start_zone
        end = self._graph.end_zone
        assert start is not None
        assert end is not None

        best = dijkstra(self._graph, start, end)
        if best is None:
            raise RuntimeError(
                "No accessible path from start to end zone."
            )
        min_cost = best.total_cost

        k = max(20, len(self._drones) * 3)
        all_paths = find_k_shortest_paths(
            self._graph, start, end, k=k,
        )
        optimal = [p for p in all_paths if p.total_cost == min_cost]
        return optimal if optimal else [best]

    def _assign(
        self,
        paths: list[Path],
        capacities: list[int],
    ) -> None:
        load = [0] * len(paths)

        for drone in self._drones:
            idx = self._best_path_index(paths, capacities, load)
            group = load[idx] // capacities[idx]
            departure = group
            load[idx] += 1

            drone.path_zones = list(paths[idx].zones)
            drone.path_connections = list(paths[idx].connections)
            drone.path_index = 1
            drone.departure_turn = departure
            drone.status = DroneStatus.WAITING

    @staticmethod
    def _best_path_index(
        paths: list[Path],
        capacities: list[int],
        load: list[int],
    ) -> int:
        def _wait(i: int) -> int:
            w = (load[i] // capacities[i])
            return w

        best = 0
        best_score = _wait(0)
        for i in range(1, len(paths)):
            score = _wait(i)
            if score < best_score:
                best_score = score
                best = i
        return best

    @staticmethod
    def _path_capacity(path: Path) -> int:
        """Return the bottleneck capacity along a path."""
        min_cap = float('inf')
        for zone in path.zones[1:-1]:
            min_cap = min(min_cap, zone.max_drones)
        for conn in path.connections:
            min_cap = min(min_cap, conn.max_link_capacity)
        return int(min_cap)


class Simulation:
    """Runs the turn-by-turn drone movement simulation.

    Usage:
        turns = Simulation(graph, drones).run()
    """

    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        assert graph.end_zone is not None
        self._graph = graph
        self._drones = drones
        self._drones_per_zone: dict[str, int] = {}
        self._reserved: dict[str, int] = {}

    def _initialize_drones_per_zone(self) -> None:
        for drone in self._drones:
            n = drone.current_zone.name
            self._drones_per_zone[n] = self._drones_per_zone.get(n, 0) + 1

    def _run_turn(
        self, active: list[Drone]
    ) -> list[TurnAction]:
        in_transit = [d for d in active if d.is_in_transit()]
        ready = [
            d for d in active
            if not d.is_in_transit()
            and self._current_turn_reached(d)
        ]

        incoming: dict[str, int] = {}
        outgoing: dict[str, int] = {}
        actions: list[TurnAction] = []

        self._resolve_transit(in_transit, incoming, actions)
        self._resolve_moves(ready, incoming, outgoing, actions)
        self._flush_occupancy(incoming, outgoing)
        return actions

    def _resolve_transit(
        self,
        drones: list[Drone],
        incoming: dict[str, int],
        actions: list[TurnAction],
    ) -> None:
        for drone in drones:
            dest = drone.transit_dest
            assert dest is not None
            if not dest.is_end:
                self._reserved[dest.name] -= 1
                if self._reserved[dest.name] == 0:
                    del self._reserved[dest.name]
            if not dest.is_end:
                incoming[dest.name] = incoming.get(dest.name, 0) + 1
            drone.current_zone = dest
            drone.status = DroneStatus.MOVING
            drone.transit_conn = None
            drone.transit_dest = None
            drone.advance()
            if dest.is_end:
                drone.status = DroneStatus.DELIVERED
            actions.append(TurnAction(drone, dest))

    def _resolve_moves(
        self,
        drones: list[Drone],
        incoming: dict[str, int],
        outgoing: dict[str, int],
        actions: list[TurnAction],
    ) -> None:
        link_usage: dict[str, int] = {}

        for drone in drones:
            dest = drone.next_zone()
            if dest is None:
                continue
            conn = drone.next_conn()
            conn_key = conn.key()

            if not self._link_free(conn, conn_key, link_usage):
                continue
            if not self._zone_free(dest, incoming, outgoing):
                continue

            if dest.movement_cost() == 2:
                self._start_transit(
                    drone, dest, conn, conn_key,
                    outgoing, link_usage, actions,
                )
            else:
                self._do_move(
                    drone, dest, conn, conn_key,
                    incoming, outgoing, link_usage, actions,
                )

    def _start_transit(
        self,
        drone: Drone,
        dest: Zone,
        conn: Connection,
        conn_key: str,
        outgoing: dict[str, int],
        link_usage: dict[str, int],
        actions: list[TurnAction],
    ) -> None:
        cur = drone.current_zone.name
        outgoing[cur] = outgoing.get(cur, 0) + 1
        link_usage[conn_key] = link_usage.get(conn_key, 0) + 1

        lbl = (conn.transit_label(drone.current_zone))
        drone.transit_conn = conn
        if not dest.is_end:
            self._reserved[dest.name] = (
                self._reserved.get(dest.name, 0) + 1
            )
        drone.transit_dest = dest
        drone.status = DroneStatus.IN_TRANSIT
        actions.append(TurnAction(drone, dest, lbl))

    def _do_move(
        self,
        drone: Drone,
        dest: Zone,
        conn: Connection,
        conn_key: str,
        incoming: dict[str, int],
        outgoing: dict[str, int],
        link_usage: dict[str, int],
        actions: list[TurnAction],
    ) -> None:
        cur = drone.current_zone.name
        outgoing[cur] = outgoing.get(cur, 0) + 1
        incoming[dest.name] = incoming.get(dest.name, 0) + 1
        link_usage[conn_key] = link_usage.get(conn_key, 0) + 1
        drone.current_zone = dest
        drone.advance()
        drone.status = (
            DroneStatus.DELIVERED if dest.is_end else DroneStatus.MOVING
        )
        actions.append(TurnAction(drone, dest))

    def _link_free(
        self,
        conn: Optional[Connection],
        conn_key: str,
        link_usage: dict[str, int],
    ) -> bool:
        if conn is None:
            return True
        return link_usage.get(conn_key, 0) < conn.max_link_capacity

    def _zone_free(
        self,
        dest: Zone,
        incoming: dict[str, int],
        outgoing: dict[str, int],
    ) -> bool:
        if dest.is_end or dest.is_start:
            return True
        current = self._drones_per_zone.get(dest.name, 0)
        going_out = outgoing.get(dest.name, 0)
        coming_in = incoming.get(dest.name, 0)
        reserved = self._reserved.get(dest.name, 0)
        return (current - going_out + coming_in + reserved) < dest.max_drones

    def _flush_occupancy(
        self,
        incoming: dict[str, int],
        outgoing: dict[str, int],
    ) -> None:
        for name, count in outgoing.items():
            self._drones_per_zone[name] = max(
                0, self._drones_per_zone.get(name, 0) - count
            )
        for name, count in incoming.items():
            self._drones_per_zone[name] = (
                self._drones_per_zone.get(name, 0) + count
            )

    def _current_turn_reached(self, drone: Drone) -> bool:
        return self._turn >= drone.departure_turn

    def run(self) -> list[list[TurnAction]]:
        """Execute the simulation and return per-turn action lists."""
        self._initialize_drones_per_zone()
        all_turns: list[list[TurnAction]] = []
        max_turns = 10 * len(self._drones) * len(self._graph.zones)

        for turn in range(max_turns):
            self._turn = turn
            active = [d for d in self._drones if not d.is_delivered()]
            if not active:
                break
            actions = self._run_turn(active)
            if actions:
                actions.sort(key=lambda a: a.drone.drone_id)
                all_turns.append(actions)
        return all_turns


def build_and_run(
    graph: Graph,
    drones: list[Drone],
) -> list[list[TurnAction]]:
    """Assign paths and run the complete simulation.

    Args:
        graph: The parsed drone network.
        drones: List of drones to route (all starting at graph.start_zone).

    Returns:
        list[list[TurnAction]]: Turn-by-turn actions for all drones.

    Raises:
        RuntimeError: If no path exists from start to end.
    """
    try:
        RoutePlanner(graph, drones).plan()
    except RuntimeError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Failed to assign paths: {exc}") from exc

    try:
        return Simulation(graph, drones).run()
    except Exception as exc:
        raise RuntimeError(f"Simulation error: {exc}") from exc
