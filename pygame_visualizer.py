"""FLY-IN · Blueprint Drone Routing Visualizer (Elite Mission Control Edition).

Blueprint theme: Advanced Holographic Aerospace Mission Control Interface.
Controls: SPACE pause | R restart | ↑↓ speed | scroll/drag=zoom/pan
| Q quit
"""
import math
import time
from typing import Optional
import pygame

from models import Graph, Zone, ZoneType
from simulator import TurnAction


RGB = tuple[int, int, int]
Snap = dict[int, tuple[float, float, float, float, bool, bool]]

BG = (0, 0, 0)
PANEL = (10, 16, 28)
SEP = (20, 38, 64)
SEP2 = (45, 75, 115)
WHITE = (230, 242, 255)
WHITE2 = (160, 185, 215)
WHITE3 = (85, 110, 140)
GREY = (70, 70, 70)
ORANGE = (255, 130, 0)
ORANGE2 = (170, 75, 0)
BLUE = (0, 190, 255)
BLUE2 = (0, 100, 175)
GREEN = (40, 225, 115)
RED = (255, 55, 55)
YELLOW = (255, 210, 50)

DRONE_COLS = [
    (255, 130, 0), (0, 190, 255), (40, 225, 115),
    (255, 210, 50), (195, 90, 255), (255, 75, 75),
    (0, 230, 195), (220, 170, 255)
]

_NAMED: dict[str, RGB] = {
    "black":   (0,   0,   0),   "maroon":  (128,  0,   0),
    "red":     (255, 70,  70),  "green":   (60,  220, 120),
    "blue":    (80,  160, 255), "yellow":  (255, 220,  50),
    "orange":  (255, 150,  40), "purple":  (200, 100, 255),
    "cyan":    (40,  210, 220), "magenta": (255,  80, 200),
    "white":   (210, 225, 255), "gray":    (130, 150, 180),
    "pink":    (255, 140, 180), "lime":    (130, 240,  80),
    "teal":    (40,  200, 180), "indigo":  (120, 100, 255),
    "brown":   (180, 110,  60), "gold":    (255, 200,  50),
    "crimson": (220,  20,  60), "darkred": (139,   0,   0),
}

ROT_SPEED = 2


def _color_from_zone(z: Zone) -> Optional[RGB]:
    """Return an explicit RGB colour for a zone if its colour name is known.

    Args:
        z: The zone to inspect.

    Returns:
        Optional[RGB]: Mapped colour tuple, or None if unknown/absent.
    """
    name = getattr(z, 'color', None)
    return _NAMED.get(name.lower(), None) if name else None


def zcol(z: Zone) -> RGB:
    """Return border_colour for a zone node.

    Args:
        z: The zone to colour.

    Returns:
        RGB: colour.
    """
    explicit = _color_from_zone(z)
    if explicit:
        return explicit
    if z.is_start:
        return ORANGE
    if z.is_end:
        return RED
    if z.zone_type == ZoneType.PRIORITY:
        return GREEN
    if z.zone_type == ZoneType.RESTRICTED:
        return YELLOW
    if z.zone_type == ZoneType.BLOCKED:
        return WHITE3
    return BLUE2


def dcol(did: int) -> RGB:
    """Return the display colour for a drone by its ID.

    Args:
        did: The drone's numeric ID (1-based).

    Returns:
        RGB: Colour tuple selected from the palette.
    """
    return DRONE_COLS[(did - 1) % len(DRONE_COLS)]


def lerp(a: float, b: float, t: float) -> float:
    """Linearly interpolate between a and b by factor t.

    Args:
        a: Start value.
        b: End value.
        t: Interpolation factor in [0, 1].

    Returns:
        float: Interpolated value.
    """
    return a + (b - a) * t


class Camera:
    """Manages pan and zoom for the world-to-screen transform.

    Attributes:
        x: Horizontal pan offset in pixels.
        y: Vertical pan offset in pixels.
        zoom: Pixels per world unit.
    """

    def __init__(self) -> None:
        """Initialise camera at origin with default zoom."""
        self.x = self.y = 0.0
        self.zoom = 50.0
        self._drag: Optional[tuple[int, int]] = None
        self._d0 = (0.0, 0.0)

    def w2s(
            self, wx: float, wy: float) -> tuple[int, int]:
        """Convert world coordinates to screen coordinates.

        Args:
            wx: World x coordinate.
            wy: World y coordinate.

        Returns:
            tuple[int, int]: Screen (x, y) pixel position.
        """
        return (int(wx * self.zoom + self.x), int(wy * self.zoom + self.y))

    def fit(
        self, xs: list[int], ys: list[int], vw: int, vh: int
    ) -> None:
        """Fit all world points into the viewport with padding.

        Args:
            xs: World x coordinates of all nodes.
            ys: World y coordinates of all nodes.
            vw: Viewport width in pixels.
            vh: Viewport height in pixels.
        """
        if not xs:
            return
        rw = (max(xs) - min(xs)) or 1.0
        rh = (max(ys) - min(ys)) or 1.0
        self.zoom = min(vw / (rw * 1.4), vh / (rh * 1.4))
        self.x = vw / 2 - ((min(xs) + max(xs)) / 2) * self.zoom
        self.y = vh / 2 - ((min(ys) + max(ys)) / 2) * self.zoom

    def zoom_at(self, vx: int, vy: int, f: float) -> None:
        """Zoom towards a viewport point by factor f.

        Args:
            vx: Viewport x of the zoom anchor.
            vy: Viewport y of the zoom anchor.
            f: Zoom scale factor (>1 zooms in, <1 zooms out).
        """
        nz = max(1, min(500.0, self.zoom * f))
        self.x = vx - (vx - self.x) * (nz / self.zoom)
        self.y = vy - (vy - self.y) * (nz / self.zoom)
        self.zoom = nz

    def start_drag(self, p: tuple[int, int]) -> None:
        """Record the start of a mouse-drag pan.

        Args:
            p: Mouse position at drag start (viewport coordinates).
        """
        self._drag = p
        self._d0 = (self.x, self.y)

    def drag(self, p: tuple[int, int]) -> None:
        """Update pan offset during a mouse drag.

        Args:
            p: Current mouse position (viewport coordinates).
        """
        if self._drag:
            self.x = self._d0[0] + (p[0] - self._drag[0])
            self.y = self._d0[1] + (p[1] - self._drag[1])

    def end_drag(self) -> None:
        """End the active mouse-drag pan."""
        self._drag = None


class Drone:
    """Animated drone sprite for the visualizer.

    Attributes:
        did: Drone numeric ID (1-based).
        col: Display colour.
        label: Human-readable label string.
        x: Current world x position.
        y: Current world y position.
        heading: Current facing angle in radians.
        target_heading: Desired facing angle.
        wait_timer: Remaining real-time seconds in the wait phase.
        waiting: True while the drone is pausing after arrival.
        is_rotating: True while the drone is aligning its heading.
        delivered: True once the drone has reached the end zone.
        pending_delivery: True when delivery will be confirmed on arrival.
        in_transit: True when traversing a restricted zone.
        p: Visibility flag.
    """

    def __init__(self, did: int, wx: float, wy: float) -> None:
        """Initialise a drone sprite at a world position.

        Args:
            did: Drone numeric ID (1-based).
            wx: Initial world x coordinate.
            wy: Initial world y coordinate.
        """
        self.did = did
        self.col = dcol(did)
        self.x = wx
        self.y = wy
        self.heading = 0.0
        self.target_heading = 0.0
        self.wait_timer = 0.0
        self.waiting = False
        self.is_rotating = False
        self.delivered = False
        self.pending_delivery = False
        self.in_transit = False
        self._fx = wx
        self._fy = wy
        self._tx = wx
        self._ty = wy
        self._t = 1.0
        self.next_heading = 0.0

    def move(
        self,
        fx: float, fy: float,
        tx: float, ty: float,
        transit: bool = False,
        next_heading: Optional[float] = None,
    ) -> None:
        """Begin animated movement from (fx, fy) to (tx, ty).

        Args:
            fx: World x of the origin position.
            fy: World y of the origin position.
            tx: World x of the destination position.
            ty: World y of the destination position.
            transit: True if this hop crosses a restricted zone.
            next_heading: If provided, the drone pre-aligns to this angle
                          during its arrival wait phase.
        """
        self._fx, self._fy = fx, fy
        self._tx, self._ty = tx, ty
        self._t = 0.0
        self.in_transit = transit
        self.waiting = False
        self.is_rotating = False

        self.next_heading = (
            next_heading if next_heading is not None
            else self.target_heading
        )

    def tick(self, dt: float, speed: float) -> None:
        """Advance the drone animation by dt seconds at the given speed.

        Phases in order:
          1. Waiting (post-arrival pause, optional heading pre-alignment).
          2. Pre-rotation (align heading before translating).
          3. Translation (linear interpolation to destination).

        Args:
            dt: Elapsed real time in seconds since the last frame.
            speed: Animation speed multiplier.
        """
        pi = math.pi
        two_pi = 2 * pi
        snap_thresh = 0.01

        def _rotate_toward(target: float, dt_: float) -> None:
            """Rotate self.heading toward target by at most ROT_SPEED*dt_.

            Args:
                target: Desired heading in radians.
                dt_: Elapsed time slice in seconds.
            """
            diff = ((target - self.heading + pi) % two_pi) - pi
            if abs(diff) <= snap_thresh:
                self.heading = target
            else:
                step = min(abs(diff), ROT_SPEED * dt_)
                self.heading += math.copysign(step, diff)

        if self.waiting:
            self.is_rotating = True
            _rotate_toward(self.next_heading, dt)
            self.wait_timer += dt * speed
            if 1.0 <= self.wait_timer:
                self.waiting = False
                self.target_heading = self.next_heading
            return

        diff = ((self.target_heading - self.heading + pi) % two_pi) - pi
        if abs(diff) > snap_thresh:
            _rotate_toward(self.target_heading, dt)
            return

        self.is_rotating = False

        if self._t < 1.0:
            self._t = min(1.0, self._t + dt * speed)
            self.x = lerp(self._fx, self._tx, self._t)
            self.y = lerp(self._fy, self._ty, self._t)

            if self._t == 1.0:
                self.x, self.y = self._tx, self._ty
                self.waiting = True
                self.wait_timer = 0.0
                if self.pending_delivery:
                    self.delivered = True
                    self.pending_delivery = False

    def moving(self) -> bool:
        """True while the drone is translating or rotating.

        Returns:
            bool: Animation in progress.
        """
        return self._t < 1.0 or self.is_rotating


class PygameVisualizer:
    """Turn-by-turn drone routing visualizer using pygame.

    Renders the zone graph, animated drones, and a telemetry panel.
    All simulation turns are pre-computed; the visualizer only animates.

    Attributes:
        BH: Height of the bottom telemetry panel in pixels.
    """

    BH = 200

    def __init__(
        self, graph: Graph, turns: list[list[TurnAction]]
    ) -> None:
        """Initialise the visualizer with a graph and pre-computed turns.

        Args:
            graph: The drone network graph.
            turns: Ordered list of simulation turns, each a list of
                   TurnAction objects.
        """
        self._g = graph
        self._turns = turns
        self._total = len(turns)
        self._idx = 0
        self._paused = False
        self._finished = False
        self._speed = 1.0
        self._drones: dict[int, Drone] = {}
        self._glow: dict[frozenset[str], float] = {}
        self._log: list[tuple[str, RGB]] = []
        self._snaps: list[Snap] = []
        self._cam = Camera()
        self._dragging = False
        self._scr: Optional[pygame.Surface] = None
        self._clock: Optional[pygame.time.Clock] = None
        self._fm: Optional[pygame.font.Font] = None
        self._fl: Optional[pygame.font.Font] = None

    def run(self) -> None:
        """Open the pygame window and enter the main event loop.

        Blocks until the user quits (Q key or window close).
        """
        pygame.init()
        self._scr = pygame.display.set_mode(
            (1540, 880), pygame.RESIZABLE
        )
        pygame.display.set_caption(
            "FLY-IN  ·  MISSION CONTROL  ·  ELECTRONIC UAV ROUTING MATRIX"
        )
        self._clock = pygame.time.Clock()
        self._fm = pygame.font.SysFont("monospace", 13, bold=True)
        self._fl = pygame.font.SysFont("monospace", 17, bold=True)

        assert self._scr is not None
        assert self._clock is not None
        assert self._fm is not None
        assert self._fl is not None
        assert self._g.start_zone is not None
        assert self._g.end_zone is not None

        self._build_snaps()
        self._init_drones()
        self._fit()
        self._apply(0)
        for d in self._drones.values():
            dx, dy = d._tx - d._fx, d._ty - d._fy
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                d.target_heading = math.atan2(dy, dx)
        clock = self._clock

        while True:
            dt = clock.tick(60) / 1000.0
            if not self._events():
                break

            for d in self._drones.values():
                d.tick(dt, self._speed)

            if not self._paused and not self._finished:
                if all(
                    not d.moving() and not d.waiting
                    for d in self._drones.values()
                ):

                    if self._idx < self._total - 1:
                        self._idx += 1
                        self._apply(self._idx)
                    else:
                        self._finished = True
                        msg = (
                            "All drones reached their destinations."
                        )
                        self._log_add(msg, GREEN)

            self._draw()

        pygame.quit()

    def _vp(self) -> pygame.Rect:
        """Return the viewport rect (window minus bottom panel).

        Returns:
            pygame.Rect: The drawable area above the telemetry panel.
        """
        scr = self._scr
        assert scr is not None
        w, h = scr.get_size()
        return pygame.Rect(0, 0, w, h - self.BH)

    def _ws(self, wx: float, wy: float) -> tuple[int, int]:
        """Convert world coords to screen coords within the viewport.

        Args:
            wx: World x coordinate.
            wy: World y coordinate.

        Returns:
            tuple[int, int]: Screen pixel position.
        """
        return self._cam.w2s(wx, wy)

    def _fit(self) -> None:
        """Fit the camera to all zone positions in the viewport."""
        vp = self._vp()
        xs = [z.x for z in self._g.zones.values()]
        ys = [z.y for z in self._g.zones.values()]
        self._cam.fit(xs, ys, vp.width, vp.height)

    def _build_snaps(self) -> None:
        """Pre-compute per-turn drone position snapshots.

        Each snapshot maps drone ID → (fx, fy, tx, ty, in_transit,
        is_delivered).  Transit hops are split at the midpoint so the
        animation moves to the centre on turn T and the destination on
        turn T+1.
        """
        assert self._g.start_zone is not None
        assert self._g.end_zone is not None

        end_name = self._g.end_zone.name
        sx0 = float(self._g.start_zone.x)
        sy0 = float(self._g.start_zone.y)
        pos: dict[int, tuple[float, float]] = {
            i: (sx0, sy0) for i in range(1, self._g.nb_drones + 1)
        }
        done: set[int] = set()

        for turn in self._turns:
            snap: Snap = {
                i: (
                    pos[i][0], pos[i][1],
                    pos[i][0], pos[i][1],
                    False, i in done,
                )
                for i in range(1, self._g.nb_drones + 1)
            }
            for a in turn:
                did = a.drone.drone_id
                tr = bool(a.conn_label)
                dz = a.moved_to
                tx, ty = float(dz.x), float(dz.y)
                fx, fy = pos.get(did, (sx0, sy0))
                if tr:
                    mx, my = (fx + tx) / 2, (fy + ty) / 2
                    snap[did] = (fx, fy, mx, my, True, False)
                    pos[did] = (mx, my)
                else:
                    is_del = dz.name == end_name
                    if is_del:
                        done.add(did)
                    snap[did] = (fx, fy, tx, ty, False, is_del)
                    pos[did] = (tx, ty)
            self._snaps.append(snap)

    def _init_drones(self) -> None:
        """Create Drone sprites at the start zone position."""
        assert self._g.start_zone is not None
        sx = float(self._g.start_zone.x)
        sy = float(self._g.start_zone.y)
        for i in range(1, self._g.nb_drones + 1):
            self._drones[i] = Drone(i, sx, sy)

    def _apply(self, idx: int) -> None:
        """Apply snapshot idx to all drone sprites and reset glow.

        Args:
            idx: Index into self._snaps to apply.
        """
        if idx >= len(self._snaps):
            return
        snap = self._snaps[idx]
        next_snap = (
            self._snaps[idx + 1] if idx + 1 < len(self._snaps) else None
        )

        for did, (fx, fy, tx, ty, tr, is_del) in snap.items():

            d = self._drones[did]

            next_hdg: Optional[float] = None
            if next_snap and did in next_snap:
                nfx, nfy, ntx, nty, *_ = next_snap[did]
                ndx, ndy = ntx - nfx, nty - nfy
                if abs(ndx) > 0.01 or abs(ndy) > 0.01:
                    next_hdg = math.atan2(ndy, ndx)

            d.move(fx, fy, tx, ty, tr, next_heading=next_hdg)

            if is_del and not d.delivered:
                d.pending_delivery = True
                self._log_add(
                    f"Drone {did} reached the end zone.", GREEN
                )

    def _restart(self) -> None:
        """Reset the visualizer to turn 0."""
        assert self._g.start_zone is not None
        self._idx = 0
        self._finished = False
        self._log.clear()
        sx = self._g.start_zone.x
        sy = self._g.start_zone.y
        for d in self._drones.values():
            d.x = d._fx = d._tx = sx
            d.y = d._fy = d._ty = sy
            d._t = 1.0
            d.delivered = d.in_transit = d.pending_delivery = False
        self._apply(0)
        for d in self._drones.values():
            dx, dy = d._tx - d._fx, d._ty - d._fy
            if abs(dx) > 0 or abs(dy) > 0:
                d.target_heading = math.atan2(dy, dx)
        self._log_add("SYSTEM REBOOT COMPLETE", ORANGE)

    def _zat(self, x: float, y: float) -> Optional[Zone]:
        """Return the zone whose world position is exactly (x, y).

        Args:
            x: World x coordinate.
            y: World y coordinate.

        Returns:
            Optional[Zone]: Matching zone, or None if not found.
        """
        for z in self._g.zones.values():
            if z.x - x == 0 and z.y - y == 0:
                return z
        return None

    def _log_add(self, msg: str, col: RGB) -> None:
        """Prepend a timestamped entry to the comms log.

        Args:
            msg: Log message text.
            col: Display colour for the message.
        """
        self._log.insert(0, (f"{time.strftime('%H:%M:%S')} -> {msg}", col))
        if len(self._log) > 12:
            self._log.pop()

    def _txt(
        self,
        text: str,
        x: int,
        y: int,
        font: pygame.font.Font,
        col: RGB,
    ) -> None:
        """Blit a text string to the screen surface.

        Args:
            text: String to render.
            x: Screen x position.
            y: Screen y position.
            font: pygame Font to use.
            col: Text colour.
        """
        scr = self._scr
        assert scr is not None

        scr.blit(font.render(text, True, col), (x, y))

    def _events(self) -> bool:
        """Process all pending pygame events.

        Returns:
            bool: False if the application should quit, True otherwise.
        """
        vp = self._vp()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_q:
                    return False
                elif k == pygame.K_SPACE:
                    self._paused = not self._paused
                elif k == pygame.K_r:
                    self._restart()
                elif k == pygame.K_UP:
                    self._speed = min(10.0, self._speed * 1.5)
                elif k == pygame.K_DOWN:
                    self._speed = max(0.2, self._speed / 1.5)
                elif k == pygame.K_f:
                    self._fit()
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                vx = ev.pos[0] - vp.x
                vy = ev.pos[1] - vp.y
                if ev.button == 1 and vp.collidepoint(ev.pos):
                    self._dragging = True
                    self._cam.start_drag((vx, vy))
                elif ev.button == 4 and vp.collidepoint(ev.pos):
                    self._cam.zoom_at(vx, vy, 1.12)
                elif ev.button == 5 and vp.collidepoint(ev.pos):
                    self._cam.zoom_at(vx, vy, 1 / 1.12)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    self._dragging = False
                    self._cam.end_drag()
            elif ev.type == pygame.MOUSEMOTION and self._dragging:
                self._cam.drag(
                    (ev.pos[0], ev.pos[1])
                )
            elif ev.type == pygame.VIDEORESIZE:
                self._fit()
        return True

    def _draw(self) -> None:
        """Render one complete frame to the screen."""
        scr = self._scr
        assert scr is not None
        w, h = scr.get_size()
        vp = self._vp()
        scr.fill(BG)
        pygame.draw.rect(scr, BG, vp)
        scr.set_clip(vp)

        self._draw_edges()
        self._draw_nodes()
        self._draw_drones()
        scr.set_clip(None)
        self._draw_bottom(w, h)

        pygame.draw.line(
            scr, ORANGE, (0, h - self.BH), (w, h - self.BH), 2
        )
        pygame.display.flip()

    def _draw_edges(self) -> None:
        """Draw all zone-to-zone connections onto the viewport."""
        scr = self._scr
        assert scr is not None
        for conn in self._g.connections:
            za, zb = conn.zone_a, conn.zone_b
            ax, ay = self._ws(za.x, za.y)
            bx, by = self._ws(zb.x, zb.y)
            is_blk = (
                za.zone_type == ZoneType.BLOCKED
                or zb.zone_type == ZoneType.BLOCKED
            )
            is_rst = (
                za.zone_type == ZoneType.RESTRICTED
                or zb.zone_type == ZoneType.RESTRICTED
            )

            if is_blk:
                mx = (ax + bx) // 2
                my = (ay + by) // 2
                pygame.draw.line(scr, GREY, (ax, ay), (bx, by), 3)
                pygame.draw.circle(scr, RED, (mx, my), 12, 2)
                pygame.draw.line(
                    scr, RED, (mx - 7, my - 7), (mx + 7, my + 7), 3
                )
                pygame.draw.line(
                    scr, RED, (mx + 7, my - 7), (mx - 7, my + 7), 3
                )
                continue

            pygame.draw.line(
                scr,
                YELLOW if is_rst else SEP2,
                (ax, ay), (bx, by), 3,
            )

    def _draw_nodes(self) -> None:
        """Draw all zone nodes with labels, capacity bars, and glow."""
        scr, fm = self._scr, self._fm
        assert scr is not None
        assert fm is not None

        zdrones: dict[str, list[Drone]] = {}
        for d in self._drones.values():
            z = self._zat(d.x, d.y)
            if z:
                zdrones.setdefault(z.name, []).append(d)

        r = max(20, min(42, int(self._cam.zoom)))

        for name, zone in self._g.zones.items():
            node_bg = BG
            if zone.color and zone.color.lower() == "black":
                node_bg = GREY
            zx, zy = self._ws(zone.x, zone.y)
            border = zcol(zone)

            cnt = len(zdrones.get(name, []))
            cap = zone.max_drones
            is_full = (
                not zone.is_start and not zone.is_end
                and cap > 0 and cnt >= cap
            )

            pygame.draw.circle(scr, node_bg, (zx, zy), r + 3)

            if is_full:
                pulse_r = r + int(5 + 3 * math.sin(time.time() * 10))
                pygame.draw.circle(scr, RED, (zx, zy), pulse_r, 2)
            pygame.draw.circle(scr, border, (zx, zy), r * .4)
            pygame.draw.circle(scr, border, (zx, zy), r + 4, 2)

            lbl = fm.render(name, False, RED if is_full else WHITE)
            offset = -r - 20 if cnt else 0
            scr.blit(
                lbl,
                (zx - lbl.get_width() // 2,
                 zy - lbl.get_height() // 2 + offset),
            )

            if not zone.is_start and not zone.is_end and cap > 0:
                bx, by = zx - r, zy + r + 10
                bw, bh = r * 2, 5
                pygame.draw.rect(
                    scr, BG, (bx, by, bw, bh), border_radius=3
                )
                if cnt > 0:
                    fw = int(bw * min(cnt / cap, 1))
                    pygame.draw.rect(
                        scr,
                        RED if is_full else border,
                        (bx, by, fw, bh),
                        border_radius=3,
                    )
                pygame.draw.rect(
                    scr, SEP2, (bx, by, bw, bh), 1, border_radius=3
                )
                ct = fm.render(f"{cnt}/{cap}", True, WHITE)
                scr.blit(ct, (zx - ct.get_width() // 2, by + 8))

    def _draw_drones(self) -> None:
        """Draw all drone sprites, grouping overlapping positions."""

        vp = self._vp()
        r = max(8, min(16, int(self._cam.zoom)))

        groups: dict[tuple[int, int], list[Drone]] = {}
        for d in self._drones.values():
            groups.setdefault(
                self._ws(d.x, d.y), []
            ).append(d)

        for (sx, sy), group in groups.items():
            for i, d in enumerate(group):
                gx, gy = sx, sy
                if len(group) > 1:
                    a = 2 * math.pi * i / len(group)
                    gx += int(math.cos(a) * r * 1.9)
                    gy += int(math.sin(a) * r * 1.9)
                if vp.collidepoint(gx, gy):
                    self._drone_dot(d, gx, gy, r)

    def _drone_dot(
        self, d: Drone, sx: int, sy: int, r: int
    ) -> None:
        """Render a single drone sprite at the given screen position.

        Draws four arms, spinning blades, a body core, heading indicator,
        and a label.

        Args:
            d: The Drone to render.
            sx: Screen x of the drone centre.
            sy: Screen y of the drone centre.
            r: Sprite radius in pixels.
        """
        scr = self._scr
        fm = self._fm
        assert scr is not None
        assert fm is not None

        heading = d.heading
        arm_len = r * 1.3
        blade_len = r * 0.6
        spin_angle = time.time() * 20.0
        for i in range(4):
            angle = heading + math.pi / 4 + i * math.pi / 2
            ax = sx + int(arm_len * math.cos(angle))
            ay = sy + int(arm_len * math.sin(angle))
            pygame.draw.line(scr, d.col, (sx, sy), (ax, ay), 4)
            pygame.draw.circle(scr, WHITE3, (ax, ay), 2)
            b_angle = spin_angle + (i * math.pi / 2)
            bx1 = ax + int(blade_len * math.cos(b_angle))
            by1 = ay + int(blade_len * math.sin(b_angle))
            bx2 = ax - int(blade_len * math.cos(b_angle))
            by2 = ay - int(blade_len * math.sin(b_angle))
            pygame.draw.line(scr, WHITE, (bx1, by1), (bx2, by2), 1)

        core_col: RGB = (50, 15, 15)
        pygame.draw.circle(scr, core_col, (sx, sy), int(r * 0.6))
        pygame.draw.circle(scr, d.col, (sx, sy), int(r * 0.6), 2)
        node_x = sx + int(r * 0.4 * math.cos(heading))
        node_y = sy + int(r * 0.4 * math.sin(heading))
        pygame.draw.circle(scr, WHITE, (node_x, node_y), 3)
        lbl = fm.render(f"D{d.did}", True, d.col)
        scr.blit(lbl, (sx - lbl.get_width() // 2, sy - r - 13))

    def _draw_bottom(self, w: int, h: int) -> None:
        """Render the four-column telemetry panel at the bottom.

        Columns: playback controls / scrubber, clock & metrics,
        mission profile, comms log.

        Args:
            w: Total window width in pixels.
            h: Total window height in pixels.
        """
        scr = self._scr
        fm = self._fm
        fl = self._fl
        clock = self._clock
        assert scr is not None
        assert fm is not None
        assert fl is not None
        assert clock is not None
        assert self._g.start_zone is not None
        assert self._g.end_zone is not None

        ty = h - self.BH
        pygame.draw.rect(scr, PANEL, (0, ty, w, self.BH))

        nd = self._g.nb_drones
        deliv = sum(1 for d in self._drones.values() if d.delivered)
        turn_d = min(self._idx + 1, self._total)
        total_moves = sum(len(t) for t in self._turns)
        margin = 16

        c1 = w // 4
        c2 = c1 * 2
        c3 = c1 * 3
        for cx in (c1, c2, c3):
            pygame.draw.line(scr, SEP2, (cx, ty + 6), (cx, h - 6), 2)

        x, y = margin, ty + margin
        bar_w = c1 - margin * 2

        if self._finished:
            sc: RGB = GREEN
            st = "COMPLETE"
        elif self._paused:
            sc = YELLOW
            st = "PAUSED"
        else:
            sc = BLUE
            st = "LIVE"
        self._txt(st, x, y, fl, sc)

        tc = fm.render(f"T -> {turn_d} / {self._total}", True, WHITE)
        scr.blit(tc, (x + bar_w - tc.get_width(), y + 4))

        y += 24
        cell_w = (bar_w - self._total) // self._total
        required_width = self._total * cell_w + (self._total - 1)
        cell_h = 12
        for i in range(self._total):
            if required_width <= 0:
                scr.blit(
                    fm.render(
                        "The number of turns is large", False, RED
                    ),
                    (x, y),
                )
                break
            cx_cell = x + i * (cell_w + 1)

            if i < self._idx:
                pygame.draw.rect(
                    scr, ORANGE2, (cx_cell, y, cell_w, cell_h)
                )
            elif i == self._idx:
                pygame.draw.rect(
                    scr, ORANGE, (cx_cell, y, cell_w, cell_h)
                )
                pygame.draw.rect(
                    scr, WHITE, (cx_cell, y, cell_w, cell_h), 1
                )
            else:
                pygame.draw.rect(
                    scr, BG, (cx_cell, y, cell_w, cell_h)
                )
                pygame.draw.rect(
                    scr, SEP, (cx_cell, y, cell_w, cell_h), 1
                )

        y += 18
        pygame.draw.rect(scr, BG,   (x, y, bar_w, 6))
        pygame.draw.rect(scr, SEP2, (x, y, bar_w, 6), 1)
        if deliv > 0:
            pygame.draw.rect(
                scr, GREEN, (x, y, int(bar_w * deliv / nd), 6)
            )
        _pct = int(100 * deliv / max(nd, 1))
        self._txt(
            f"ARRIVALS  {deliv}/{nd}  ({_pct}%)", x, y + 9, fm, WHITE2
        )

        kb = fm.render(
            "SPC:PAUSE R:REBOOT  ↑↓:SPEED  F:FIT Q:QUIT",
            True, WHITE,
        )
        scr.blit(kb, (x, h - 24))

        x = c1 + margin
        bar_w2 = c2 - c1 - margin * 2
        mid = c1 + (c2 - c1) // 2

        clock_str = time.strftime("%H:%M:%S")
        clk = fl.render(clock_str, True, WHITE)
        clock_y = ty + margin
        scr.blit(clk, (mid - clk.get_width() // 2, clock_y))

        rule_y = clock_y + clk.get_height() + 6
        pygame.draw.line(
            scr, SEP2, (x, rule_y), (x + bar_w2, rule_y), 2
        )

        def _metric(
            label: str, value: str, row_y: int, vc: RGB
        ) -> None:
            """Render a left-label / right-value metric row.

            Args:
                label: Metric name string.
                value: Metric value string.
                row_y: Screen y for this row.
                vc: Colour for the value text.
            """
            lbl_s = fm.render(label, True, WHITE3)
            val_s = fm.render(value, True, vc)
            scr.blit(lbl_s, (x, row_y))
            scr.blit(val_s, (x + bar_w2 - val_s.get_width(), row_y))

        ry = rule_y + 10
        _metric("ZOOM   ", f"{self._cam.zoom:.0f} px", ry, BLUE)
        ry += 28
        _metric("SPEED  ", f"× {self._speed:.1f}", ry, ORANGE)
        ry += 28
        _metric("FPS    ", f"{clock.get_fps():.0f}", ry, WHITE2)

        x, y = c2 + margin, ty + margin
        self._txt("MISSION PROFILE", x, y, fm, ORANGE)
        y += 20
        pygame.draw.line(scr, SEP2, (x, y), (c3 - margin, y), 2)
        y += 8

        col_w = (c3 - c2 - margin * 2) // 2 + 10
        rows_data = [
            ("ORIGIN",    self._g.start_zone.name, ORANGE),
            ("TARGET",    self._g.end_zone.name,   RED),
            ("NODES",     str(len(self._g.zones)),       WHITE),
            ("EDGES",     str(len(self._g.connections)), WHITE),
            ("FLEET",     str(nd),                       BLUE),
            ("MOVES",     str(total_moves),              WHITE),
            ("AVG/DRONE", f"{total_moves / max(nd, 1):.1f}", WHITE),
        ]
        for row_lbl, val, vc in rows_data:
            if y > h - 20:
                break
            scr.blit(fm.render(row_lbl, True, WHITE3), (x, y))
            scr.blit(fm.render(val, True, vc), (x + col_w, y))
            y += 14

        y += 4
        if y < h - 36:
            sep_w = c3 - c2 - margin * 2
            pygame.draw.line(scr, SEP, (x, y), (x + sep_w, y), 2)
            y += 6
        if int(time.time()) % 2:
            scr.blit(fm.render("SYSTEM ONLINE", True, GREEN), (x, y))

        x, y = c3 + margin, ty + margin
        if x + 60 < w:
            self._txt("COM-LINK STREAM", x, y, fm, ORANGE)
            y += 20
            pygame.draw.line(scr, SEP2, (x, y), (w - margin, y), 2)
            y += 6
            for msg, mc in self._log:
                if y > h - 16:
                    break
                scr.blit(fm.render(msg, True, mc), (x, y))
                y += 13
