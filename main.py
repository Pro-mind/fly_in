"""Fly-in: Drone Routing Simulation — main entry point.

Usage:
    python main.py <map_file>

Official simulation output (turn-by-turn) is written to stdout.
"""

import sys

from models import Drone, Graph
from parser import parse_map, ParseError
from simulator import build_and_run, TurnAction


class Simulation:
    """Orchestrates parsing, routing, output, and visualisation.

    Attributes:
        filepath: Path to the drone network map file.
    """

    def __init__(self, filepath: str) -> None:
        """Initialise a Simulation.

        Args:
            filepath: Path to the drone network map file.
        """
        self.filepath = filepath
        self._graph: Graph
        self._drones: list[Drone]
        self._turns: list[list[TurnAction]]

    def run(self) -> None:
        """Parse map, run simulation, emit all output."""
        self._load()
        self._simulate()
        self._print_header()
        self._print_summary()
        self._print_official_output()
        self._launch_gui()

    def _load(self) -> None:
        """Parse the map file and initialise drones.

        Raises:
            SystemExit: On parse or file-not-found errors.
        """
        try:
            self._graph = parse_map(self.filepath)
        except ParseError as exc:
            print(f"[ERROR] {exc}", file=sys.stderr)
            sys.exit(1)
        except FileNotFoundError as exc:
            print(f"[ERROR] {exc}", file=sys.stderr)
            sys.exit(1)

        assert self._graph.start_zone is not None
        assert self._graph.end_zone is not None

        self._drones = [
            Drone(drone_id=i + 1, start_zone=self._graph.start_zone)
            for i in range(self._graph.nb_drones)
        ]

    def _simulate(self) -> None:
        """Assign paths and execute the simulation.

        Raises:
            SystemExit: On routing or simulation errors.
        """
        try:
            self._turns = build_and_run(self._graph, self._drones)
        except RuntimeError as exc:
            print(f"[ERROR] {exc}")
            sys.exit(1)

    def _print_header(self) -> None:
        """Print a styled header summarising the loaded map."""

        g = self._graph
        assert g.start_zone is not None
        assert g.end_zone is not None

        print('─' * 60)
        print("FLY-IN · DRONE ROUTING SIMULATION")
        print('─' * 60)
        print(
            f"  Map        : {g.start_zone.name}"
            f" → {g.end_zone.name}"
        )
        print(f"  Drones     : {g.nb_drones}")
        print(
            f"  Zones      : {len(g.zones)}  "
            f"  Connections: {len(g.connections)}\n"
        )

    def _print_summary(self) -> None:
        """Print a summary block after all turns are done."""

        nb = self._graph.nb_drones
        total_moves = sum(len(t) for t in self._turns)
        print('─' * 60)
        print("  Simulation complete")
        print(f"  Total turns : {len(self._turns)}")
        print(f"  Drones      : {nb}")
        print(f"  Total moves : {total_moves}",)
        if nb > 0:
            avg = total_moves / nb
            print(f"  Avg moves/drone : {avg:.1f}")
        print('─' * 60)

    def _print_official_output(self) -> None:
        """Print the official stdout output"""
        for actions in self._turns:
            if not actions:
                continue
            tokens: list[str] = []
            for action in actions:
                dest = (
                    action.conn_label
                    if action.conn_label
                    else action.moved_to.name
                )
                tokens.append(f"D{action.drone.drone_id}-{dest}")
            print(" ".join(tokens))

    def _launch_gui(self) -> None:
        """Launch the pygame visualizer."""
        try:
            from pygame_visualizer import PygameVisualizer
            PygameVisualizer(self._graph, self._turns).run()
        except ImportError:
            print(
                "[WARN] pygame not found — install it:"
                " pip install pygame"
            )
        except Exception as exc:
            print(f"[WARN] pygame visualizer error: {exc}")


def main() -> None:
    """Parse CLI arguments and launch the simulation."""
    if len(sys.argv) != 2:
        print("Usage: python main.py <map_file>")
        sys.exit(1)

    filepath = sys.argv[1]
    Simulation(filepath).run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye")
