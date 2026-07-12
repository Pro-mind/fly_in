*This project has been created as part of the 42 curriculum by [sbarbaq].*

---

# Fly-in — Drone Routing Simulation

## Description

**Fly-in** is a Python simulation system that routes a fleet of autonomous drones through a network of connected zones, moving them all from a central start hub to a target end hub in the minimum number of simulation turns.

The network is a weighted directed graph where each zone carries a movement cost and an occupancy limit, and each connection carries a link-capacity limit. The simulation must respect all constraints simultaneously: no zone may hold more drones than its `max_drones` limit, no link may carry more simultaneous crossings than its `max_link_capacity`, restricted zones require a mandatory two-turn crossing, and blocked zones are entirely inaccessible.

The program reads a plain-text map file, computes optimal routing for every drone, executes the turn-by-turn simulation, and prints a styled header, the official turn-by-turn output, and a summary to the terminal (`stdout`), while parsing and simulation errors are reported on `stderr`. A full-featured graphical visualizer (pygame) is also provided for replaying the result.

---

## Instructions

### Requirements

- Python 3.10+
- `pygame` — graphical visualizer
- `flake8`, `mypy` — linting (required by the 42 norm)

### Install dependencies

```bash
pip install pygame flake8 mypy
```

Or via the Makefile:

```bash
make install
```

### Run a simulation

```bash
python main.py <map_file>
```

The program always prints the header, official turn-by-turn output, and
summary to the terminal first, then attempts to launch the pygame graphical
replay. If `pygame` is not installed, or the visualizer raises any error, a
warning is printed and the program exits cleanly — the terminal output is
never affected (there is no `--no-gui` flag; both behave as one sequential
pipeline).

Example with the provided map:

```bash
python main.py map.txt
```

Or via the Makefile (defaults to `map.txt`, override with `MAP=<file>`):

```bash
make run
make run MAP=other_map.txt
make debug            # run under pdb for step-by-step debugging
```

### Lint

Or via the Makefile:

```bash
make lint          # flake8 + mypy (warn-return-any, disallow-untyped-defs, ...)
make lint-strict    # flake8 + mypy --strict
```

Equivalent direct commands:

```bash
flake8 .
mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports \
     --disallow-untyped-defs --check-untyped-defs
mypy . --strict
```

### Map file format

```
nb_drones: <positive integer>

start_hub: <name> <x> <y> [optional metadata]
end_hub:   <name> <x> <y> [optional metadata]
hub:       <name> <x> <y> [optional metadata]

connection: <zoneA>-<zoneB> [optional metadata]
```

Zone metadata keys: `zone=(normal|blocked|restricted|priority)`, `color=<name>`, `max_drones=<int>`.  
Connection metadata keys: `max_link_capacity=<int>`.

Example:

```
nb_drones: 7

start_hub: start 0 0
end_hub: goal 6 0 [color=green]

hub: A 1 -2 [max_drones=1]
hub: B 3 -2 [color=red max_drones=1]
hub: C 4  0 [max_drones=3]

connection: start-A [max_link_capacity=1]
connection: A-B
connection: B-C
connection: C-goal
```

---

## Project Structure

```
fly-in/
├── main.py              Entry point — Simulation class + CLI
├── models.py            Data models: Zone, Connection, Drone, Graph
├── parser.py            Map file parser with full error reporting
├── pathfinder.py        Dijkstra + Yen's k-shortest paths
├── simulator.py         Path assignment + turn-by-turn simulation engine
├── pygame_visualizer.py Graphical visualizer (pygame)
├── Makefile             install / run / debug / clean / lint / lint-strict
├── map.txt              Example map
└── README.md
```

---

## Algorithm Choices and Implementation Strategy

### 1. Pathfinding — Dijkstra

The core pathfinder is a standard **Dijkstra's algorithm** implemented from scratch using Python's `heapq` min-heap. It computes the minimum-cost path from the start hub to the end hub, where cost is the sum of `movement_cost()` of each destination zone along the path:

- `normal` → cost 1
- `priority` → cost 1 (preferred via secondary tiebreak score)
- `restricted` → cost 2
- `blocked` → never entered (filtered in `Graph.neighbors()`)

A secondary tiebreak key (sum of zone priority scores) is stored alongside the main cost in the heap so that among equal-cost paths, those passing through `PRIORITY` zones are always preferred. This is done without modifying the primary cost, keeping the algorithm correct.

### 2. Multi-Path Generation — Yen's k-Shortest Paths

To give the scheduler real routing choices, **Yen's algorithm** enumerates up to `k` distinct shortest paths. The algorithm iterates over every spur node on each confirmed shortest path, temporarily banning the edges already used by confirmed paths sharing the same root prefix, and runs Dijkstra from the spur node. The resulting spur path is combined with the root prefix to form a candidate; candidates are maintained in a min-heap and deduplicated before being confirmed.

This avoids the pathological case of routing all drones on a single bottleneck path when equally short alternatives exist.

### 3. Route Assignment — Capacity-Weighted Staggered Departure

Once the set of optimal paths (those matching the minimum cost) is known, drones are distributed across them:

1. **Path throughput** is computed as the minimum `max_drones` across all intermediate zones and `max_link_capacity` across all connections on the path — the maximum number of drones that can pipeline through simultaneously.
2. Drones are assigned to the path that minimises their **effective wait time**: `(drones_already_assigned // throughput) * stagger_interval`, ensuring high-capacity paths absorb more drones before forcing others to wait.
3. **Departure stagger**: consecutive drone groups on the same path are spaced by the maximum movement cost along that path (1 for all-normal paths, 2 if any restricted zone is present). This guarantees no two drones from the same group ever compete for the same intermediate zone.

Drones are **never** routed onto longer paths when optimal paths can still accept them — they simply depart later — so the total turn count is never inflated by an unnecessary detour.

### 4. Simulation Engine — Turn-by-Turn Conflict Resolution

Each simulation turn is processed in two strict phases:

**Phase 1 — mandatory transit arrivals.** Drones that began a restricted-zone crossing in the previous turn are committed to arriving this turn regardless of current occupancy. This matches the subject rule that a drone on a restricted link cannot stop mid-crossing.

**Phase 2 — regular moves.** Drones that are not already in transit and whose scheduled departure turn has been reached are processed in drone-ID order. For each drone:
- The target link's current usage this turn is checked against `max_link_capacity`.
- The target zone's *effective* occupancy is computed as `current − outgoing_this_turn + incoming_this_turn + reserved_this_turn`, where `reserved_this_turn` accounts for drones that committed to a restricted-zone crossing toward that same destination in a previous turn and are guaranteed to land there next turn. This lets drones vacating a zone free capacity for incoming drones in the same turn, while still respecting spots already claimed by in-flight transits.
- If either check fails the drone simply waits; no re-routing occurs.
- If the destination is a restricted zone, the drone enters a two-turn `IN_TRANSIT` state and outputs the connection label (e.g. `D1-hub-roof1`) this turn; it will arrive unconditionally next turn.

Occupancy counters are flushed at the end of each turn, and the per-turn action list is re-sorted by drone ID before being appended to the output (internal processing order and final output order are independent). The simulation terminates as soon as all drones have reached the end hub. A safety cap of `10 × nb_drones × |zones|` turns prevents infinite loops on degenerate inputs.

### Complexity

| Phase              | Complexity                        |
|--------------------|-----------------------------------|
| Dijkstra           | O((V + E) log V)                  |
| Yen's k paths      | O(k · V · (V + E) log V)          |
| Path assignment    | O(D · P) where D=drones, P=paths  |
| Simulation         | O(turns × D) ≈ O(N²) worst case  |

Paths are computed once before the simulation starts; there is no per-turn replanning.

---

## Visual Representation

### Pygame Visualizer (`pygame_visualizer.py`)

The primary visualizer is a full interactive graphical window built with `pygame`. It provides a real-time animated replay of the simulation with the following features:

**Map rendering**
- Zones are drawn as filled circles whose color encodes their type (orange = start, red = end, green = priority, yellow = restricted, grey = blocked) or their explicit `color=` metadata.
- A soft glow ring is drawn behind each zone. Zones at full capacity pulse with a red outline to signal congestion at a glance.
- A capacity bar below each zone shows the live drone count relative to `max_drones`.
- Active edges animate with a bright blue glow that fades over ~1.3 seconds after a drone traverses them; blocked edges render a red ✕ crosshair.

**Drone rendering**
- Each drone is drawn as a small quadcopter sprite: four arms radiating from a core, each tipped with a spinning rotor blade that animates in real time.
- Drones rotate smoothly toward their heading before beginning linear motion, giving a physically plausible flight feel.
- When multiple drones share the same zone they are fanned out in a circle to remain individually visible.
- Delivered drones orbit the end hub.

**Bottom telemetry panel**
- Left column: turn scrubber bar (tap any cell to jump), delivery progress bar, and keybindings.
- Centre column: live wall clock, zoom level, playback speed, and FPS.
- Right columns: mission profile (origin, target, fleet size, total moves) and a timestamped COM-LINK log of delivery events.

**Interaction**

| Key / gesture | Action |
|---|---|
| `SPACE` | Pause / resume |
| `R` | Restart from turn 0 |
| `↑` / `↓` | Increase / decrease playback speed |
| `F` | Fit all zones into the viewport |
| Scroll wheel | Zoom in / out at cursor |
| Click + drag | Pan the map |
| `Q` | Quit |

These features allow a user to immediately verify correctness — capacity violations, unexpected waiting, or missed deliveries are instantly visible — and to present the simulation result to peers or evaluators in a clear, engaging format.

### Terminal Output

Before the graphical window opens, the program prints a plain-text header and a closing summary block around the official turn-by-turn output on `stdout`: the header shows the start/end zone names, drone count, and network size; the summary shows total turns, total moves, and average moves per drone. Any parsing or simulation failure (bad map syntax, missing file, unreachable end hub) is reported on `stderr` instead, so error messages are never mixed in with valid simulation output.

---

## Resources

- [Dijkstra's Algorithm — Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [Yen's k-Shortest Paths Algorithm — Wikipedia](https://en.wikipedia.org/wiki/Yen%27s_k-shortest_path_algorithm)
- [Python `heapq` — standard library documentation](https://docs.python.org/3/library/heapq.html)
- [PEP 257 — Docstring Conventions](https://peps.python.org/pep-0257/)
- [pygame documentation](https://www.pygame.org/docs/)
- [mypy documentation](https://mypy.readthedocs.io/)
- [flake8 documentation](https://flake8.pycqa.org/)
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
