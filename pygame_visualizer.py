"""FLY-IN · Blueprint Drone Routing Visualizer (Elite Mission Control Edition)
Blueprint theme: Advanced Holographic Aerospace Mission Control Interface.
Controls: SPACE pause | R restart | ←→ step | ↑↓ speed | scroll/drag=zoom/pan
| Q quit
"""
from __future__ import annotations
import math
import time
from typing import Optional
import pygame

try:
    from models import Graph, Zone, ZoneType
    from simulator import TurnAction
except ImportError as e:
    raise ImportError(
        f"Needs models.py / simulator.py in same folder: {e}") from e

# ── Types ──────────────────────────────────────────────────────────────
RGB = tuple[int, int, int]
Snap = dict[int, tuple[float, float, float, float, bool, bool]]

# ── Palette ─────────────────────────────────────────────────────────────
BG = (3, 5, 10)
BG2 = (0, 0, 0)
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


def _color_from_zone(z: Zone) -> Optional[RGB]:
    name = getattr(z, 'color', None)
    return _NAMED.get(name.lower(), None) if name else None


def zcol(z: Zone) -> tuple[RGB, RGB]:
    explicit = _color_from_zone(z)
    if explicit:
        return explicit, BG2
    if z.is_start:
        return ORANGE, BG2
    if z.is_end:
        return RED, BG2
    if z.zone_type == ZoneType.PRIORITY:
        return GREEN, BG2
    if z.zone_type == ZoneType.RESTRICTED:
        return YELLOW, BG2
    if z.zone_type == ZoneType.BLOCKED:
        return WHITE3, BG2
    return BLUE2, BG2


def dcol(did: int) -> RGB:
    return DRONE_COLS[(did - 1) % len(DRONE_COLS)]


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def hex_pts(cx: int, cy: int, r: int) -> list[tuple[int, int]]:
    return [(int(cx + r * math.cos(math.radians(60 * i))),
             int(cy + r * math.sin(math.radians(60 * i)))) for i in range(6)]


# ── Camera Matrix ───────────────────────────────────────────────────────
class Camera:
    def __init__(self) -> None:
        self.x = self.y = 0.0
        self.zoom = 50.0
        self._drag: Optional[tuple[int, int]] = None
        self._d0 = (0.0, 0.0)

    def w2s(self, wx: float, wy: float, ox: int, oy: int) -> tuple[int, int]:
        return (
            int(wx * self.zoom + self.x) + ox,
            int(wy * self.zoom + self.y) + oy,
        )

    def fit(self, xs: list[float], ys: list[float], vw: int, vh: int) -> None:
        if not xs:
            return
        rw = (max(xs) - min(xs)) or 1.0
        rh = (max(ys) - min(ys)) or 1.0
        self.zoom = min(vw / (rw * 1.4), vh / (rh * 1.4))

        self.x = vw / 2 - ((min(xs) + max(xs)) / 2) * self.zoom
        self.y = vh / 2 - ((min(ys) + max(ys)) / 2) * self.zoom

    def zoom_at(self, vx: int, vy: int, f: float) -> None:
        nz = min(400.0, self.zoom * f)
        self.x = vx - (vx - self.x) * (nz / self.zoom)
        self.y = vy - (vy - self.y) * (nz / self.zoom)
        self.zoom = nz

    def start_drag(self, p: tuple[int, int]) -> None:
        self._drag = p
        self._d0 = (self.x, self.y)

    def drag(self, p: tuple[int, int]) -> None:
        if self._drag:
            self.x = self._d0[0] + (p[0] - self._drag[0])
            self.y = self._d0[1] + (p[1] - self._drag[1])

    def end_drag(self) -> None:
        self._drag = None


# ── Drone Asset ─────────────────────────────────────────────────────────
# ── Constants (place near top of file, after palette) ──────────────────
ROT_SPEED = 2   # radians per second of real time (tune freely)


# ── Drone Asset ─────────────────────────────────────────────────────────
class Drone:
    def __init__(self, did: int, wx: float, wy: float) -> None:
        self.did = did
        self.col = dcol(did)
        self.label = f"UAV-{did}"
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
        self.p = True

    def move(
        self,
        fx: float, fy: float,
        tx: float, ty: float,
        transit: bool = False,
        next_heading: Optional[float] = None,
    ) -> None:
        self._fx, self._fy = fx, fy
        self._tx, self._ty = tx, ty
        self._t = 0.0
        self.in_transit = transit
        self.waiting = False
        self.is_rotating = False

        dx, dy = tx - fx, ty - fy
        if abs(dx) > 0.01 or abs(dy) > 0.01:
            self.target_heading = math.atan2(dy, dx)

        if next_heading is not None:
            self.pre_align_heading = next_heading
        else:
            self.pre_align_heading = self.target_heading

    def tick(self, dt: float, speed: float) -> None:
        PI = math.pi
        TWO_PI = 2 * PI
        SNAP_THRESH = 0.01  # radians — snap when this close

        def _rotate_toward(target: float, dt_: float) -> None:
            """Rotate self.heading toward target by at most ROT_SPEED*dt_."""
            diff = ((target - self.heading + PI) % TWO_PI) - PI
            if abs(diff) <= SNAP_THRESH:
                self.heading = target
            else:
                step = min(abs(diff), ROT_SPEED * dt_)
                self.heading += math.copysign(step, diff)

        # ── 1. Waiting phase ──────────────────────────────────────────────
        if self.waiting:
            self.is_rotating = True
            _rotate_toward(self.pre_align_heading, dt)
            self.wait_timer -= dt * speed
            if self.wait_timer <= 0:
                self.waiting = False
                self.is_rotating = False
                self.target_heading = self.pre_align_heading
            return
        # ── 2. Pre-rotation phase (align before moving) ────────────────
        diff = ((self.target_heading - self.heading + PI) % TWO_PI) - PI
        if abs(diff) > SNAP_THRESH:
            self.is_rotating = True
            _rotate_toward(self.target_heading, dt)
            return

        self.is_rotating = False

        # ── 3. Translation phase ───────────────────────────────────────
        if self._t < 1.0:
            self._t = min(1.0, self._t + dt * speed)
            self.x = lerp(self._fx, self._tx, self._t)
            self.y = lerp(self._fy, self._ty, self._t)

            if self._t >= 1.0:
                self.x, self.y = self._tx, self._ty
                self.waiting = True
                self.wait_timer = 0.45      # seconds of real time
                if self.pending_delivery:
                    self.delivered = True
                    self.pending_delivery = False

    @property
    def moving(self) -> bool:
        return self._t < 1.0 or self.is_rotating


# ── Visualizer Core Engine ──────────────────────────────────────────────
class PygameVisualizer:
    BH = 200  # bottom panel height

    def __init__(self, graph: Graph, turns: list[list[TurnAction]]) -> None:
        self._g = graph
        self._turns = turns
        self._total = len(turns)
        self._idx = 0
        self._paused = False
        self._finished = False
        self._speed = 1.0
        self._timer = 0.0
        self._drones: dict[int, Drone] = {}
        self._glow: dict[frozenset[str], float] = {}
        self._log: list[tuple[str, RGB]] = []
        self._snaps: list[Snap] = []
        self._cam = Camera()
        self._dragging = False
        self._scr: Optional[pygame.Surface] = None
        self._clock: Optional[pygame.time.Clock] = None
        self._fs: Optional[pygame.font.Font] = None
        self._fm: Optional[pygame.font.Font] = None
        self._fl: Optional[pygame.font.Font] = None

    def run(self) -> None:
        pygame.init()
        self._scr = pygame.display.set_mode((1440, 880), pygame.RESIZABLE)
        pygame.display.set_caption(
            "FLY-IN  ·  MISSION CONTROL  ·  ELECTRONIC UAV ROUTING MATRIX"
        )
        self._clock = pygame.time.Clock()
        self._fs = pygame.font.SysFont("monospace", 11)
        self._fm = pygame.font.SysFont("monospace", 13, bold=True)
        self._fl = pygame.font.SysFont("monospace", 17, bold=True)
        self._build_snaps()
        self._init_drones()
        self._fit()
        self._apply(0)

        clock = self._clock

        while True:

            dt = min(clock.tick(60) / 1000.0, 0.05)
            if not self._events():
                break

            for d in self._drones.values():
                d.tick(dt, self._speed * 2.2)

            if not self._paused and not self._finished:
                if all(
                    not d.moving and not d.waiting
                    for d in self._drones.values()
                ):
                    self._timer += dt
                    if self._timer >= max(0.2, 0.75 / self._speed):
                        self._timer = 0.0
                        if self._idx < self._total - 1:
                            self._idx += 1
                            self._apply(self._idx)
                        else:
                            self._finished = True
                            msg = (
                                "OBJECTIVE ACHIEVED:"
                                " ALL RECON FLIGHTS COMPLETED"
                            )
                            self._log_add(msg, GREEN)
            for k in list(self._glow):
                self._glow[k] -= dt
                if self._glow[k] <= 0:
                    self._glow.pop(k)
            self._draw()
        pygame.quit()

    def _vp(self) -> pygame.Rect:
        scr = self._scr
        assert scr is not None
        w, h = scr.get_size()
        return pygame.Rect(0, 0, w, h - self.BH)

    def _ws(self, wx: float, wy: float) -> tuple[int, int]:
        vp = self._vp()
        return self._cam.w2s(wx, wy, vp.x, vp.y)

    def _fit(self) -> None:
        vp = self._vp()
        xs = [float(z.x) for z in self._g.zones.values()]
        ys = [float(z.y) for z in self._g.zones.values()]
        self._cam.fit(xs, ys, vp.width, vp.height)

    def _build_snaps(self) -> None:
        assert self._g.start_zone is not None
        assert self._g.end_zone is not None
        end_name = self._g.end_zone.name
        sx0, sy0 = float(self._g.start_zone.x), float(self._g.start_zone.y)
        pos = {i: (sx0, sy0) for i in range(1, self._g.nb_drones + 1)}
        done: set[int] = set()
        for turn in self._turns:
            snap = {
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
        assert self._g.start_zone is not None
        sx, sy = float(self._g.start_zone.x), float(self._g.start_zone.y)
        for i in range(1, self._g.nb_drones + 1):
            self._drones[i] = Drone(i, sx, sy)

    def _apply(self, idx: int) -> None:
        if idx >= len(self._snaps):
            return
        snap = self._snaps[idx]
        next_snap = (
            self._snaps[idx + 1] if idx + 1 < len(self._snaps) else None
        )
        self._glow.clear()

        for did, (fx, fy, tx, ty, tr, is_del) in snap.items():
            if did not in self._drones:
                continue
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
                self._log_add(f"UAV-{did} VECTOR DESTINATION REACHED", GREEN)

            if fx != tx or fy != ty:
                # look up zones by name from the turn action, not by coord
                turn_actions = (
                    self._turns[idx] if idx < len(self._turns) else []
                )
                za_name = zb_name = None
                for a in turn_actions:
                    if a.drone.drone_id == did:
                        zb_name = a.moved_to.name
                        # from-zone: find which zone sits at (fx, fy)
                        za = self._zat(fx, fy)
                        za_name = za.name if za else None
                        break
                if za_name and zb_name:
                    self._glow[frozenset([za_name, zb_name])] = 1.3
                else:
                    # fallback for transit midpoints
                    za, zb = self._zat(fx, fy), self._zat(tx, ty)
                    if za and zb:
                        self._glow[frozenset([za.name, zb.name])] = 1.3

    def _restart(self) -> None:
        assert self._g.start_zone is not None
        self._idx = 0
        self._finished = False
        self._timer = 0.0
        self._glow.clear()
        self._log.clear()
        sx, sy = float(self._g.start_zone.x), float(self._g.start_zone.y)
        for d in self._drones.values():
            d.x = d._fx = d._tx = sx
            d.y = d._fy = d._ty = sy
            d._t = 1.0
            d.delivered = d.in_transit = d.pending_delivery = False
        self._apply(0)
        self._log_add("SYSTEM THERMAL RESET REBOOT COMPLETE", ORANGE)

    def _zat(self, x: float, y: float) -> Optional[Zone]:
        for z in self._g.zones.values():
            if z.x - x == 0 and z.y - y == 0:
                return z
        return None

    def _log_add(self, msg: str, col: RGB) -> None:
        self._log.insert(0, (f"{time.strftime('%H:%M:%S')} ❯ {msg}", col))
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
        scr = self._scr
        if scr is None:
            return
        s = font.render(text, True, col)

        scr.blit(s, (x, y))

    def _events(self) -> bool:
        vp = self._vp()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
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
                vx, vy = ev.pos[0] - vp.x, ev.pos[1] - vp.y
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
                vp2 = self._vp()
                self._cam.drag((ev.pos[0] - vp2.x, ev.pos[1] - vp2.y))
            elif ev.type == pygame.VIDEORESIZE:
                self._fit()
        return True

    def _draw(self) -> None:
        scr = self._scr
        assert scr is not None
        w, h = scr.get_size()
        vp = self._vp()
        scr.fill(BG)
        pygame.draw.rect(scr, BG2, vp)
        scr.set_clip(vp)

        self._draw_edges()
        self._draw_nodes()
        self._draw_drones()
        scr.set_clip(None)
        self._draw_bottom(w, h)

        pygame.draw.line(scr, ORANGE, (0, h - self.BH), (w, h - self.BH), 2)
        pygame.display.flip()

    def _draw_edges(self) -> None:
        scr = self._scr
        assert scr is not None
        for conn in self._g.connections:
            za, zb = conn.zone_a, conn.zone_b
            ax, ay = self._ws(float(za.x), float(za.y))
            bx, by = self._ws(float(zb.x), float(zb.y))
            glow = self._glow.get(frozenset([za.name, zb.name]), 0.0)
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
                pygame.draw.line(scr, GREY, (ax, ay), (bx, by), 2)
                pygame.draw.circle(scr, RED, (mx, my), 12)
                pygame.draw.circle(scr, BG, (mx, my), 10)
                pygame.draw.line(
                    scr, RED, (mx - 7, my - 7), (mx + 7, my + 7), 3)
                pygame.draw.line(
                    scr, RED, (mx + 7, my - 7), (mx - 7, my + 7), 3)

                continue

            if glow > 0.01:
                g_factor = min(1.0, glow)
                col = tuple(
                    max(0, min(255, int(lerp(BLUE2[i], BLUE[i], g_factor))))
                    for i in range(3)
                )
                pygame.draw.line(
                    scr,
                    (col[0] // 4, col[1] // 4, col[2] // 4),
                    (ax, ay), (bx, by), int(2 + glow * 4) + 4,
                )
                pygame.draw.line(
                    scr, col, (ax, ay), (bx, by),
                    max(2, int(1 + glow * 3)),
                )
            else:
                pygame.draw.line(
                    scr,
                    YELLOW if is_rst else SEP2,
                    (ax, ay), (bx, by), 1,
                )

    def _draw_nodes(self) -> None:
        scr, fs, fm = self._scr, self._fs, self._fm
        assert scr and fs and fm

        zdrones: dict[str, list[Drone]] = {}

        for d in self._drones.values():
            z = self._zat(d.x, d.y)
            if z:
                zdrones.setdefault(z.name, []).append(d)
        r = max(20, min(42, int(self._cam.zoom * .58)))

        for name, zone in self._g.zones.items():

            sx, sy = self._ws(float(zone.x), float(zone.y))
            border, fill = zcol(zone)

            cnt = len(zdrones.get(name, []))
            cap = zone.max_drones

            is_full = (
                not zone.is_start and not zone.is_end
                and cap > 0 and cnt >= cap
            )

            glow_col = (border[0]//6, border[1]//6, border[2]//6)

            pygame.draw.circle(scr, glow_col, (sx, sy), r+14)

            if is_full:
                pulse_r = r + int(5 + 3*math.sin(time.time()*10))
                pygame.draw.circle(scr, RED, (sx, sy), pulse_r, 2)

            pygame.draw.circle(scr, border, (sx, sy), r + 4, 2)
            pygame.draw.circle(scr, fill, (sx, sy), r)

            core_col = (
                min(255, border[0]+40),
                min(255, border[1]+40),
                min(255, border[2]+40)
            )

            pygame.draw.circle(scr, core_col, (sx, sy), int(r*.4))

            lbl = fm.render(name, False, RED if is_full else WHITE)

            offset = -r-20 if cnt else 0

            scr.blit(
                lbl,
                (sx - lbl.get_width() // 2,
                 sy - lbl.get_height() // 2 + offset)
            )

            if not zone.is_start and not zone.is_end and cap > 0:

                bx, by = sx - r, sy + r + 10
                bw, bh = r * 2, 5

                pygame.draw.rect(scr, BG, (bx, by, bw, bh), border_radius=3)

                if cnt > 0:
                    fw = int(bw * min(cnt / cap, 1))
                    pygame.draw.rect(
                        scr,
                        RED if is_full else border,
                        (bx, by, fw, bh),
                        border_radius=3
                    )

                pygame.draw.rect(
                    scr, SEP2, (bx, by, bw, bh), 1, border_radius=3)

                ct = fs.render(f"{cnt}/{cap}", True, WHITE3)

                scr.blit(ct, (sx - ct.get_width() // 2, by + 8))

    def _draw_drones(self) -> None:
        scr = self._scr
        end_zone = self._g.end_zone
        assert scr is not None and end_zone is not None
        vp = self._vp()
        r = max(8, min(16, int(self._cam.zoom * 0.22)))

        groups: dict[tuple[int, int], list[Drone]] = {}
        for d in self._drones.values():
            if not d.delivered:
                groups.setdefault(self._ws(d.x, d.y), []).append(d)

        for (sx, sy), group in groups.items():
            for i, d in enumerate(group):
                gx, gy = sx, sy
                if len(group) > 1:
                    a = 2 * math.pi * i / len(group)
                    gx += int(math.cos(a) * r * 1.5)
                    gy += int(math.sin(a) * r * 1.5)
                if vp.collidepoint(gx, gy):
                    self._drone_dot(d, gx, gy, r)

        ex, ey = self._ws(float(end_zone.x), float(end_zone.y))
        deliv_fleet = [d for d in self._drones.values() if d.delivered]
        for i, d in enumerate(deliv_fleet):
            a = 2 * math.pi * i / max(len(deliv_fleet), 1)
            dx2 = ex + int(math.cos(a) * r * 1.9)
            dy2 = ey + int(math.sin(a) * r * 1.9)
            if vp.collidepoint(dx2, dy2):
                self._drone_dot(d, dx2, dy2, r)

    def _drone_dot(self, d: Drone, sx: int, sy: int, r: int) -> None:
        scr = self._scr
        fs = self._fs
        assert scr is not None and fs is not None

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

        core_col = (50, 15, 15)

        pygame.draw.circle(scr, core_col, (sx, sy), int(r * 0.6))
        pygame.draw.circle(scr, d.col, (sx, sy), int(r * 0.6), 2)
        node_x = sx + int(r * 0.4 * math.cos(heading))
        node_y = sy + int(r * 0.4 * math.sin(heading))
        pygame.draw.circle(scr, WHITE, (node_x, node_y), 3)
        lbl = fs.render(f"D{d.did}", True, WHITE)
        scr.blit(lbl, (sx - lbl.get_width() // 2, sy - r - 13))

    # ── Bottom panel ────────────────────────────────────────────────────
    def _draw_bottom(self, w: int, h: int) -> None:
        scr = self._scr
        fs = self._fs
        fm = self._fm
        fl = self._fl
        clock = self._clock
        assert (
            scr is not None and fs is not None
            and fm is not None and fl is not None
            and clock is not None
        )
        assert self._g.start_zone is not None
        assert self._g.end_zone is not None

        ty = h - self.BH
        pygame.draw.rect(scr, PANEL, (0, ty, w, self.BH))

        nd = self._g.nb_drones
        deliv = sum(1 for d in self._drones.values() if d.delivered)
        turn_d = min(self._idx + 1, self._total)
        total_moves = sum(len(t) for t in self._turns)
        margin = 16

        # four equal columns
        C1 = w // 4
        C2 = C1 * 2
        C3 = C1 * 3
        for cx in (C1, C2, C3):
            pygame.draw.line(scr, SEP2, (cx, ty + 6), (cx, h - 6), 2)

        x, y = margin, ty + margin
        bar_w = C1 - margin * 2

        # status word
        if self._finished:
            sc, st = GREEN,  "COMPLETE"
        elif self._paused:
            sc, st = YELLOW, "PAUSED"
        else:
            sc, st = BLUE,   "LIVE"
        self._txt(st, x, y, fl, sc)

        # turn counter right-aligned in same row
        tc = fs.render(f"T -> {turn_d} / {self._total}", True, WHITE3)
        scr.blit(tc, (x + bar_w - tc.get_width(), y + 4))

        # turn scrubber
        y += 24
        cell_w = (bar_w - self._total) // max(self._total, 1)
        cell_h = 12
        for i in range(self._total):
            if cell_w <= 0:
                txt = "The number of turns is large"
                scr.blit(fs.render(txt, False, RED), (x, y))
            cx_cell = x + i * (cell_w + 1)
            if cx_cell + cell_w > x + bar_w:
                break
            if i < self._idx:
                pygame.draw.rect(scr, ORANGE2, (cx_cell, y, cell_w, cell_h))
            elif i == self._idx:
                pygame.draw.rect(scr, ORANGE,  (cx_cell, y, cell_w, cell_h))
                pygame.draw.rect(scr, WHITE,   (cx_cell, y, cell_w, cell_h), 1)
            else:
                pygame.draw.rect(scr, BG,  (cx_cell, y, cell_w, cell_h))
                pygame.draw.rect(scr, SEP, (cx_cell, y, cell_w, cell_h), 1)

        # delivery progress bar + label
        y += 18
        pygame.draw.rect(scr, BG,   (x, y, bar_w, 6))
        pygame.draw.rect(scr, SEP2, (x, y, bar_w, 6), 1)
        if deliv > 0:
            pygame.draw.rect(scr, GREEN, (x, y, int(bar_w * deliv / nd), 6))
        _pct = int(100 * deliv / max(nd, 1))
        self._txt(f"ARRIVALS  {deliv}/{nd}  ({_pct}%)", x, y + 9, fs, WHITE2)

        # keybind
        kb = fs.render(
            "SPC:PAUSE  R:REBOOT  ↑↓:SPEED  F:FIT  Q:QUIT",
            True, WHITE3,
        )
        scr.blit(kb, (x, h - 14))

        # ── COL 1 · Telemetry readouts ───────────────────────────────────
        #   Large clock  /  zoom  /  speed  /  fps
        x = C1 + margin
        bar_w2 = C2 - C1 - margin * 2          # usable width of this column
        mid = C1 + (C2 - C1) // 2           # horizontal centre of column

        # big clock — centred, prominent
        clock_str = time.strftime("%H:%M:%S")
        clk = fl.render(clock_str, True, WHITE)
        clock_y = ty + margin
        scr.blit(clk, (mid - clk.get_width() // 2, clock_y))

        # thin rule under clock
        rule_y = clock_y + clk.get_height() + 6
        pygame.draw.line(scr, SEP2, (x, rule_y), (x + bar_w2, rule_y), 2)

        # three metric rows: label left, value right
        def _metric(label: str, value: str, row_y: int, vc: RGB) -> None:
            lbl_s = fs.render(label, True, WHITE3)
            val_s = fm.render(value,  True, vc)
            scr.blit(lbl_s, (x, row_y))
            scr.blit(val_s, (x + bar_w2 - val_s.get_width(), row_y))

        ry = rule_y + 10
        ln = "-" * 31
        _metric("ZOOM   " + ln, f"{self._cam.zoom:.0f} px", ry, BLUE)
        ry += 28
        _metric("SPEED  " + ln, f"{self._speed:.1f} ×", ry, ORANGE)
        ry += 28
        _metric("FPS    " + ln, f"{clock.get_fps():.0f}", ry, WHITE2)

        # ── COL 2 · Mission profile ──────────────────────────────────────
        x, y = C2 + margin, ty + margin
        self._txt("MISSION PROFILE", x, y, fm, ORANGE)
        y += 20
        pygame.draw.line(scr, SEP2, (x, y), (C3 - margin, y), 2)
        y += 8

        col_w = (C3 - C2 - margin * 2) // 2 + 10
        rows_data = [
            ("ORIGIN", self._g.start_zone.name, ORANGE),
            ("TARGET", self._g.end_zone.name, RED),
            ("NODES", str(len(self._g.zones)), WHITE),
            ("EDGES", str(len(self._g.connections)), WHITE),
            ("FLEET", str(nd), BLUE),
            ("MOVES", str(total_moves), WHITE),
            ("AVG/DRONE", f"{total_moves / max(nd, 1):.1f}", WHITE),
        ]
        for lbl, val, vc in rows_data:
            if y > h - 20:
                break
            scr.blit(fs.render(lbl, True, WHITE3), (x, y))
            scr.blit(fs.render(val, True, vc), (x + col_w, y))
            y += 14

        y += 4
        if y < h - 36:
            sep_w = C3 - C2 - margin * 2
            pygame.draw.line(scr, SEP, (x, y), (x + sep_w, y), 2)
            y += 6
        if int(time.time()) % 2:
            scr.blit(
                fs.render("SYSTEM ONLINE", True, GREEN),
                (x, y)
            )

        # ── COL 3 · Comms log ────────────────────────────────────────────
        x, y = C3 + margin, ty + margin
        if x + 60 < w:
            self._txt("COM-LINK STREAM", x, y, fm, ORANGE)
            y += 20
            pygame.draw.line(scr, SEP2, (x, y), (w - margin, y), 2)
            y += 6
            for msg, mc in self._log:
                if y > h - 16:
                    break
                scr.blit(fs.render(msg, True, mc), (x, y))
                y += 13
