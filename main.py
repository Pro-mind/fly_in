"""Fly-in: Drone Routing Simulation — main entry point.

Usage:
    python main.py <map_file> [--no-gui]

Official simulation output (turn-by-turn) is written to stdout per VII.5.
Colored terminal representation is written to stderr.
The graphical animation window opens automatically unless --no-gui is passed.
"""

from __future__ import annotations

import sys
from typing import TextIO

from models import Drone, Graph
from parser import parse_map, ParseError
from simulator import build_and_run, TurnAction

# ---------------------------------------------------------------------------
# ANSI colour codes for terminal output
# ---------------------------------------------------------------------------

_RESET = "\033[0m"
_BOLD = "\033[1m"
_DIM = "\033[2m"


# ---------------------------------------------------------------------------
# Simulation orchestrator
# ---------------------------------------------------------------------------

class Simulation:
    """Orchestrates parsing, routing, output, and visualisation.

    Attributes:
        filepath: Path to the drone network map file.
        gui:      Whether to open the graphical visualizer window.
    """

    def __init__(self, filepath: str, gui: bool = True) -> None:
        """Initialise a Simulation.

        Args:
            filepath: Path to the drone network map file.
            gui:      Whether to open the graphical visualizer window.
        """
        self.filepath = filepath
        self.gui = gui
        self._graph: Graph
        self._drones: list[Drone]
        self._turns: list[list[TurnAction]]

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Parse map, run simulation, emit all output."""
        self._load()
        self._simulate()
        self._print_header()
        self._print_summary()
        self._print_official_output()
        if self.gui:
            self._launch_gui()

    # ------------------------------------------------------------------
    # Pipeline stages
    # ------------------------------------------------------------------

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
            print(f"[ERROR] {exc}", file=sys.stderr)
            sys.exit(1)

    # ------------------------------------------------------------------
    # Terminal output helpers
    # ------------------------------------------------------------------

    def _print_header(
        self, file: TextIO = sys.stderr
    ) -> None:
        """Print a styled header summarising the loaded map.

        Args:
            file: Output stream (default stderr).
        """
        g = self._graph
        assert g.start_zone is not None
        assert g.end_zone is not None

        sep = f"{_DIM}{'─' * 60}{_RESET}"
        print(sep, file=file)
        print(
            f"{_BOLD}\033[96m  ✦  FLY-IN · DRONE ROUTING SIMULATION"
            f"{_RESET}",
            file=file,
        )
        print(sep, file=file)
        print(
            f"  Map        : {_BOLD}{g.start_zone.name}{_RESET}"
            f" → {_BOLD}{g.end_zone.name}{_RESET}",
            file=file,
        )
        print(
            f"  Drones     : {_BOLD}{g.nb_drones}{_RESET}",
            file=file,
        )
        print(
            f"  Zones      : {len(g.zones)}  "
            f"  Connections: {len(g.connections)}",
            file=file,
        )

    def _print_summary(
        self, file: TextIO = sys.stderr
    ) -> None:
        """Print a summary block after all turns are done.

        Args:
            file: Output stream (default stderr).
        """
        nb = self._graph.nb_drones
        total_moves = sum(len(t) for t in self._turns)
        sep = f"{_DIM}{'─' * 60}{_RESET}"
        print("", file=file)
        print(sep, file=file)
        print(
            f"  {_BOLD}\033[92m✓ Simulation complete{_RESET}",
            file=file,
        )
        print(
            f"  Total turns : {_BOLD}{len(self._turns)}{_RESET}",
            file=file,
        )
        print(f"  Drones      : {_BOLD}{nb}{_RESET}", file=file)
        print(
            f"  Total moves : {_BOLD}{total_moves}{_RESET}",
            file=file,
        )
        if nb > 0:
            avg = total_moves / nb
            print(
                f"  Avg moves/drone : {_BOLD}{avg:.1f}{_RESET}",
                file=file,
            )
        print(sep, file=file)

    def _print_official_output(self) -> None:
        """Print the official stdout output per subject section VII.5."""
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

    # ------------------------------------------------------------------
    # GUI
    # ------------------------------------------------------------------

    def _launch_gui(self) -> None:
        """Launch the pygame visualizer."""
        try:
            from pygame_visualizer import PygameVisualizer
            PygameVisualizer(self._graph, self._turns).run()
        except ImportError:
            print(
                "[WARN] pygame not found — install it:"
                " pip install pygame",
                file=sys.stderr,
            )
        except Exception as exc:
            import traceback
            print(
                f"[WARN] pygame visualizer error: {exc}",
                file=sys.stderr,
            )
            traceback.print_exc(file=sys.stderr)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Parse CLI arguments and launch the simulation."""
    if len(sys.argv) not in (2, 3):
        print(
            "Usage: python main.py <map_file> [--no-gui]",
            file=sys.stderr,
        )
        sys.exit(1)

    filepath = sys.argv[1]
    gui = "--no-gui" not in sys.argv
    Simulation(filepath, gui=gui).run()


if __name__ == "__main__":
    main()
