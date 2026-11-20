"""Orchestration of the warehouse (belts, dispatcher, carters, arms, forklifts).
Hardware-agnostic: it talks to `hw` (Isaac-backed or fake) through a small interface, so the whole
control flow can be tested off-line.  Every actor is a generator advanced once per physics step."""
import math, random
import numpy as np
import config as C
from nav import Grid, Mobile, wrap

DT = 1.0 / 60.0
HARD_D, CLOSE_D, SAFE_D = 1.9, 1.35, 2.5   # recomputed from the carter radius in Sim.__init__


def PKG_H():
    return C.PKG_SIZE[2]


# unused (kept so old comments still make sense)            # centre distance at which a lower-priority carter yields


def wait(t):
    for _ in range(int(t / DT)):
        yield


class Dynamic:
    """Attribute view of a person/prop dict (see people.py) so arbitration, evasion and logging
    can treat a walking human exactly like another vehicle.  Priority 99: people always win."""

    def __init__(self, p, name):
        self.p, self.name = p, name
        self.mob = p["mob"]

    def prio(self):
        return 99


class Belt:
    def __init__(self, tag, y):
        self.tag, self.y = tag, y
        self.sgn = -1 if y > 0 else 1               # toward room centre
        self.dock = (C.PICK_X, y + self.sgn * C.DOCK_OFFSET_Y)
        self.W = (C.PICK_X - C.DOCK_APPROACH, self.dock[1])
        self.pkg = None
        self.ready = False
        self.ready_t = 0.0
        self.docked = None
        self.carter = None
        self.placed = 0
        self.roster = []          # carters that serve this belt, in turn order
        self.turn = 0
        self.line_busy = False    # True from dispatch until the forklift has finished this crate


class CarterU:
    def __init__(self, name, x, y, yaw, roamer):
        self.name, self.roamer = name, roamer
        self.mob = Mobile(x, y, yaw, C.CARTER_VMAX, C.CARTER_WMAX)
        self.job = None
        self.cargo = None
        self.parked = False
        self.park_xy = None
        self.evading = False
        self.nudge = None
        self.still_t = 0.0
        self.backoff = 0.0
        self.applied_xy = None      # last pose written to the scene (people guard)
        self._people_blocked = False

    def prio(self):
        if self.cargo is not None:
            return 3
        if self.job is not None:
            return 2
        return 0 if self.roamer else 1

    def slot_world(self):
        sx, sy = C.CARTER_SLOT
        cy, sy_ = math.cos(self.mob.yaw), math.sin(self.mob.yaw)
        return (self.mob.x + cy * sx - sy_ * sy, self.mob.y + sy_ * sx + cy * sy)


class ForkU:
    def __init__(self, name, fx, fy, g):
        self.name, self.fx, self.fy = name, fx, fy
        self.x, self.y, self.lift = fx, fy, 0.0
        self.sgn = 1 if fy > 0 else -1               # which side of the room
        self.surf = g["surf"]                        # fork top height above the lift height 0
        self.Ctip = g["tip"] - (C.PKG_SIZE[1] / 2 + 0.10)   # forklift origin -> package centre when carried
        self.cargo_yaw = 0.0
        self.owner = None
        self.carter_at_ct = None
        self.cargo = None
        self.carter_free = False
        self.delivered = 0
        self.belt = None
        self.CT = (C.CARTER_TRANSFER_X, fy)
        self.AP = (C.CARTER_TRANSFER_X, fy - self.sgn * 1.9)


class Sim:
    def __init__(self, hw, arms, seed=0, people=None):
        random.seed(seed)
        self.hw, self.arms, self.t = hw, arms, 0.0
        self.events, self.gens = [], []
        self.belts = {t: Belt(t, c["y"]) for t, c in C.CONVEYORS.items()}
        global HARD_D, CLOSE_D, SAFE_D
        r = C.CARTER_RADIUS
        HARD_D, CLOSE_D, SAFE_D = 2 * r + 0.6, 2 * r + 0.05, min(2 * r + 1.2, 2.8)
        self.forks = {}
        for n, f in C.FORKLIFTS.items():
            self.forks[n] = ForkU(n, f["pos"][0], f["pos"][1], hw.fork_geometry(n))
        self.carters = {}
        for n, (x, y) in C.CARTER_START.items():
            self.carters[n] = CarterU(n, x, y, 0.0, n == C.ROAMER)
        self.fork_of = {t: min(self.forks.values(), key=lambda f: abs(f.fy - b.y)) for t, b in self.belts.items()}
        for t, b in self.belts.items():
            b.roster = [C.CARTER_PRIMARY[t]] + ([C.ROAMER] if t == C.SUPPORT_BELT else [])
            self.fork_of[t].belt = b
        # dynamic obstacles: walking people + the shuttle pushcart (people.py), yielded to by
        # every robot - they are disks in the planner, blockers in arbitrate, never touched.
        self.people = [p for p in (people or []) if p is not None]
        self.grid = self._build_grid()
        self.min_sep = 99.0
        self.min_person_sep = 99.0

    # ------------------------------------------------------------------ setup
    def _build_grid(self):
        hx, hy = C.ROOM_X / 2, C.ROOM_Y / 2
        rects = [(-hx - 1, -hx, -hy - 1, hy + 1), (hx, hx + 1, -hy - 1, hy + 1),
                 (-hx - 1, hx + 1, hy, hy + 1), (-hx - 1, hx + 1, -hy - 1, -hy)]
        for b in self.belts.values():
            rects.append((C.BELT_START_X, C.BELT_START_X + C.BELT_LEN, b.y - 0.38, b.y + 0.38))
            py = b.y + b.sgn * C.ARM_OFFSET_Y
            rects.append((C.PICK_X - 0.2, C.PICK_X + 0.2, py - 0.2, py + 0.2))
        for f in self.forks.values():
            s = f.sgn
            x0, x1 = 5.7, 12.9
            lo, hi = f.fy - 1.0, f.fy + 1.0
            rects.append((x0, x1, lo, hi))          # whole strip is a no-go zone; CT is reached by a scripted straight move
        # Add static obstacles from scene
        for name in self.hw.static_rects() if hasattr(self.hw, 'static_rects') else []:
            pass  # already added by scene.build
        g = Grid(-hx, hx, -hy, hy, 0.1)
        r = C.CARTER_RADIUS
        rects += [tuple(x) for x in self.hw.static_rects()]
        g.build(rects, r + 0.15, 0.8 * r)
        # (people are dynamic: they enter the planner as disks in goto()/plan(), never baked in)
        self.ex_r = r + 0.8
        for f in self.forks.values():                      # keep the carter access lane clear for its owner
            ya, yb = sorted((f.fy - f.sgn * 3.6, f.fy - f.sgn * 0.9))
            m = (g.X >= C.CARTER_TRANSFER_X - r - 0.6) & (g.X <= C.CARTER_TRANSFER_X + r + 0.6) & (g.Y >= ya) & (g.Y <= yb)
            g.soft |= m
        for b in self.belts.values():                       # and the dock approach lane
            m = (g.X >= b.W[0] - 2.8) & (g.X <= b.dock[0] + 0.9) & (np.abs(g.Y - b.dock[1]) <= max(0.65, r + 0.05))
            g.soft |= m
        return g

    def log(self, msg):
        self.events.append((self.t, msg))
        print(f"[{self.t:7.1f}s] {msg}", flush=True)

    def start(self):
        for b in self.belts.values():
            self.gens.append(self.belt_proc(b))
        self.gens.append(self.dispatch_proc())
        for c in self.carters.values():
            self.gens.append(self.carter_proc(c))
        for b in self.belts.values():
            self.gens.append(self.arm_proc(b, self.arms[b.tag]))
        for f in self.forks.values():
            self.gens.append(self.fork_proc(f))

    def step(self):
        self.t += DT
        for g in list(self.gens):
            try:
                next(g)
            except StopIteration:
                self.gens.remove(g)
        cs = list(self.carters.values())
        self._deadlock_breaker(cs)
        for i in range(len(cs)):
            for j in range(i + 1, len(cs)):
                self.min_sep = min(self.min_sep, math.hypot(cs[i].mob.x - cs[j].mob.x, cs[i].mob.y - cs[j].mob.y))
        for p in self.people:                      # closest any carter ever got to a person/prop
            pm = p["mob"]
            for c in cs:
                self.min_person_sep = min(self.min_person_sep,
                                          math.hypot(c.mob.x - pm.x, c.mob.y - pm.y))

    def _deadlock_breaker(self, cs):
        """If a carter has been standing still for 8 s with another carter in its way, the junior one reverses
        straight away from the blocker (never into an obstacle) so the standoff dissolves."""
        for c in cs:
            near = [o for o in cs if o is not c and math.hypot(o.mob.x - c.mob.x, o.mob.y - c.mob.y) < HARD_D]
            waiting = abs(c.mob.v) < 0.02 and not c.parked and near
            c.still_t = c.still_t + DT if waiting else 0.0
            if c.backoff > 0:
                c.backoff -= DT
                bx = c.mob.x - 0.35 * math.cos(c.mob.yaw) * DT
                by = c.mob.y - 0.35 * math.sin(c.mob.yaw) * DT
                if any(math.hypot(bx - p["mob"].x, by - p["mob"].y)
                       < C.CARTER_RADIUS + p.get("r", 0.45) + 0.10 for p in self.people):
                    continue                        # never reverse into a person/prop
                if self.grid.free_point(bx, by, self._pdisk()) or not self.grid.free_point(c.mob.x, c.mob.y):
                    c.mob.x, c.mob.y = bx, by
                    self._apply(c)
                continue
            if c.still_t > 8.0:
                loser = min([c] + near, key=lambda o: (o.prio(), o.name), default=c)
                loser = max([c] + near, key=lambda o: (-o.prio(), o.name))
                if loser is c:
                    c.backoff, c.still_t = 3.0, 0.0
                    self.log(f"{c.name}: standoff -> backing away")

    # ------------------------------------------------------------------ belts
    def belt_proc(self, b):
        while True:
            b.pkg = self.hw.spawn_pkg(b.tag)
            b.ready = False
            self.log(f"belt {b.tag}: new package on belt")
            t0, warned = self.t, False
            while not self._at_pick(b.pkg):
                if not warned and self.t - t0 > 60.0:
                    p = b.pkg.pos()
                    self.log(f"belt {b.tag}: crate has not reached the stopper after 60 s - it is at x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}, "
                             f"expected x={C.PICK_X:.2f}, y={b.y:.2f}")
                    warned = True
                yield
            yield from wait(0.6)
            b.ready, b.ready_t = True, self.t
            self.log(f"belt {b.tag}: package waiting at stopper")
            while b.pkg is not None:
                yield
            b.ready = False
            b.placed += 1

    def _at_pick(self, pkg):
        p = pkg.pos()
        return abs(p[0] - C.PICK_X) < 0.04 and p[2] < C.BELT_H + 0.3 and pkg.settled()

    # ------------------------------------------------------------------ dispatcher
    def dispatch_proc(self):
        """Round operation: for every belt the next carter in its roster is called only when the previous crate
        has been completely handled (carter gone, forklift finished).  One comes, finishes, then the next."""
        while True:
            for b in self.belts.values():
                if not b.ready or b.line_busy or b.carter is not None or b.docked is not None:
                    continue
                c = self.carters[b.roster[b.turn % len(b.roster)]]
                if self._idle(c):
                    c.job, b.carter, b.line_busy = (b, self.fork_of[b.tag]), c, True
                    b.turn += 1
                    self.log(f"dispatch: {c.name} -> belt {b.tag} (turn {b.turn})")
            yield

    @staticmethod
    def _idle(c):
        return c.job is None and c.cargo is None

    # ------------------------------------------------------------------ carters
    def _apply(self, c):
        # People and moving props have absolute right of way: a carter never enters their disk,
        # even when a manoeuvre (turn, backoff, deadlock escape) was not people-aware.
        c._people_blocked = False
        for p in self.people:
            m = p["mob"]
            rr = C.CARTER_RADIUS + p.get("r", 0.45) + 0.10
            if math.hypot(c.mob.x - m.x, c.mob.y - m.y) < rr:
                if c.applied_xy is not None and math.hypot(c.applied_xy[0] - m.x,
                                                           c.applied_xy[1] - m.y) >= rr:
                    c.mob.x, c.mob.y = c.applied_xy
                c.mob.v = 0.0
                c._people_blocked = True
                break
        self.hw.carter_set(c.name, c.mob.x, c.mob.y, c.mob.yaw)
        if c.cargo is not None:
            sx, sy = c.slot_world()
            c.cargo.set_pose([sx, sy, C.CARTER_DECK_H + PKG_H() / 2], c.mob.yaw)
        c.applied_xy = (c.mob.x, c.mob.y)

    def _rel(self, c, o):
        """(distance, forward, lateral) of carter o in c's heading frame."""
        dx, dy = o.mob.x - c.mob.x, o.mob.y - c.mob.y
        d = math.hypot(dx, dy)
        co, si = math.cos(c.mob.yaw), math.sin(c.mob.yaw)
        return d, dx * co + dy * si, -dx * si + dy * co

    def _pdisk(self, pad=0.1):
        """People/props as planner disks (x, y, r) sized for a *carter centre* to stay clear of them."""
        return [(p["mob"].x, p["mob"].y, p.get("r", 0.45) + C.CARTER_RADIUS + pad)
                for p in self.people]

    @staticmethod
    def _on_path(c, pts, p, pad=0.2):
        """True when person/prop p is within touching distance of the remaining path polyline."""
        m = p["mob"]
        rr = C.CARTER_RADIUS + p.get("r", 0.45) + pad
        if math.hypot(m.x - c.mob.x, m.y - c.mob.y) > 5.0 + rr:
            return False
        ax, ay = c.mob.x, c.mob.y
        for bx, by in pts:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            s = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((m.x - ax) * dx + (m.y - ay) * dy) / L2))
            if math.hypot(m.x - (ax + s * dx), m.y - (ay + s * dy)) < rr:
                return True
            ax, ay = bx, by
        return False

    def arbitrate(self, c):
        """Returns (must_stop, blocker, threat).  Rank rule: lower priority yields inside SAFE_D.
        Safety rule: nobody drives into a carter that is in its path (within the swept width), whatever its rank."""
        lat = C.CARTER_RADIUS + 0.35
        blk, who, threat = False, None, None
        for o in self.carters.values():
            if o is c:
                continue
            d, f, l = self._rel(c, o)
            if d > SAFE_D:
                continue
            wins = (o.prio() > c.prio()) or (o.prio() == c.prio() and o.name < c.name)
            if wins and (abs(o.mob.v) > 0.05 or d < HARD_D):
                threat = o                                  # somebody senior is coming through
            in_path = f > 0.0 and abs(l) < lat
            if (wins and in_path) or (in_path and d < HARD_D) or (f > 0.0 and d < CLOSE_D and abs(l) < lat * 1.15):
                blk, who = True, o
        # people and moving props: absolute right of way - never touched, always yielded to
        for i, p in enumerate(self.people):
            o = Dynamic(p, p.get("name", f"dyn{i}"))
            d, f, l = self._rel(c, o)
            if d > SAFE_D + 0.6:
                continue
            lat_p = lat + p.get("r", 0.45)
            if f > -0.3 and abs(l) < lat_p and d < max(HARD_D, C.CARTER_RADIUS + p.get("r", 0.45) + 0.4):
                blk, who = True, o
                if d < 2.0 and abs(c.mob.v) > 0.05 and (abs(o.mob.v) > 0.05 or d < 1.3):
                    threat = o                        # a person walking into our path: make way
        return blk, who, threat

    def _evade(self, c, other):
        """Move to a reachable free spot away from `other`. Always yields at least once (never busy-loops)."""
        for radii, sep in (((2.5, 3.5), 2.2), ((1.8, 2.5, 3.2), 1.6), ((1.2, 1.8, 2.4), 1.1)):
            cands = []
            for k in range(16):
                ang = 2 * math.pi * k / 16
                for r in radii:
                    p = (c.mob.x + r * math.cos(ang), c.mob.y + r * math.sin(ang))
                    if not self.grid.free_point(p[0], p[1], self._pdisk()):
                        continue
                    if any(math.hypot(p[0] - o.mob.x, p[1] - o.mob.y) < sep for o in self.carters.values() if o is not c):
                        continue
                    cands.append((math.hypot(p[0] - other.mob.x, p[1] - other.mob.y) - 0.5 * r, p))
            cands.sort(reverse=True)
            for _, p in cands[:6]:
                if self.grid.plan((c.mob.x, c.mob.y), p, self._pdisk(), 1.3, 0.3):
                    self.log(f"{c.name}: making way for {other.name}")
                    c.evading = True
                    yield from self.goto(c, p, 0.4, abort=lambda: False)
                    c.evading = False
                    return
        yield from wait(0.5)            # nowhere to go right now; try again shortly

    def _escape(self, c):
        """Deadlock breaker: creep to the free spot farthest from everybody."""
        best, bs = None, -1.0
        pd = self._pdisk()
        for k in range(24):
            ang = 2 * math.pi * k / 24
            for r in (2.0, 3.0, 4.0):
                p = (c.mob.x + r * math.cos(ang), c.mob.y + r * math.sin(ang))
                if not self.grid.free_point(p[0], p[1], pd):
                    continue
                sc = min([math.hypot(p[0] - o.mob.x, p[1] - o.mob.y) for o in self.carters.values() if o is not c] + [9.0])
                if sc > bs:
                    best, bs = p, sc
        if best is None:
            return
        self.log(f"{c.name}: deadlock escape")
        t = 0.0
        while t < 10.0 and math.hypot(best[0] - c.mob.x, best[1] - c.mob.y) > 0.3:
            near = any(math.hypot(o.mob.x - c.mob.x, o.mob.y - c.mob.y) < 0.9 and
                       ((o.mob.x - c.mob.x) * math.cos(c.mob.yaw) + (o.mob.y - c.mob.y) * math.sin(c.mob.yaw)) > 0
                       for o in self.carters.values() if o is not c)
            c.mob.track(best, DT, move=not near, tol=0.3)
            self._apply(c)
            t += DT
            yield

    def goto(self, c, goal, tol=0.15, abort=None, dock=False, must=False):
        exg = self.ex_r if dock else 0.3
        fails = 0
        path, idx, stuck, threatened, replans = None, 0, 0.0, 0.0, 0
        replan_t = -99.0
        extra = []
        while True:
            if abort and abort():
                return False
            if path is None:
                disks = [(o.mob.x, o.mob.y, 0.75) for o in self.carters.values() if o is not c]
                # dynamic people/props as disks (planned clear of both surfaces)
                disks += [(p["mob"].x, p["mob"].y, p.get("r", 0.45) + C.CARTER_RADIUS + 0.1)
                          for p in self.people]
                disks += extra
                path = self.grid.plan((c.mob.x, c.mob.y), goal, disks, 1.3, exg) or self.grid.plan((c.mob.x, c.mob.y), goal, (), 1.3, exg)
                idx = 0
                if path is None:
                    fails += 1
                    if not must and fails >= 2:
                        return False
                    if fails in (1, 10, 100):
                        self.log(f"{c.name}: no path to ({goal[0]:.1f}, {goal[1]:.1f}), retrying")
                    yield from wait(1.0)
                    continue
                replan_t = self.t
            if not path:
                return True
            # dynamic avoidance: a person/prop that walked onto the remaining path forces a
            # fresh plan around its *new* position (throttled, so a crossing person just gets waited out)
            if self.t - replan_t > 0.8 and any(self._on_path(c, path[idx:], p) for p in self.people):
                path, replan_t = None, self.t
                continue
            last = idx == len(path) - 1
            if c.nudge is not None and not c.evading:
                o, c.nudge = c.nudge, None
                yield from self._evade(c, o)
                path = None
                continue
            blk, other, threat = self.arbitrate(c)
            reached = c.mob.track(path[idx], DT, move=not blk, tol=tol if last else 0.2)
            self._apply(c)
            stuck = stuck + DT if blk else 0.0
            threatened = threatened + DT if (threat is not None and not c.evading) else 0.0
            if reached:
                idx += 1
                if idx >= len(path):
                    return True
            if threatened > 0.4 and c.job is None or threatened > 1.5:
                yield from self._evade(c, threat)
                path, threatened, stuck = None, 0.0, 0.0
                yield
                continue
            if stuck > 7.0:
                yield from self._escape(c)
                path, stuck = None, 0.0
                continue
            if stuck > 3.0 and other is not None:
                extra = [(other.mob.x, other.mob.y, 1.1)]
                path, stuck, replans = None, 0.0, replans + 1
                if c.roamer and replans > 2:
                    return False
            yield

    def _hard_blocked(self, c):
        lat = C.CARTER_RADIUS + 0.35
        for o in self.carters.values():
            if o is c:
                continue
            d, f, l = self._rel(c, o)
            if d < HARD_D and f > 0.0 and abs(l) < lat:
                return True
        for i, p in enumerate(self.people):         # never drive straight into a person/prop either
            o = Dynamic(p, p.get("name", f"dyn{i}"))
            d, f, l = self._rel(c, o)
            if d < C.CARTER_RADIUS + p.get("r", 0.45) + 0.3 and f > -0.2 and abs(l) < lat + p.get("r", 0.45):
                return True
        return False

    def straight(self, c, wp, yaw_end=None, tol=0.03):
        blocked_t = 0.0
        while True:
            hb = self._hard_blocked(c)
            if c.mob.drive_straight(wp, DT, tol=tol, move=not hb):
                break
            blocked_t = blocked_t + DT if hb else 0.0
            if blocked_t > 3.0:
                for o in self.carters.values():
                    if o is not c and math.hypot(o.mob.x - c.mob.x, o.mob.y - c.mob.y) < HARD_D:
                        o.nudge = c
                blocked_t = 0.0
            self._apply(c)
            yield
        if yaw_end is not None:
            while not c.mob.turn_to(yaw_end, DT):
                self._apply(c)
                yield
        self._apply(c)

    def _alt_park(self, c):
        """Nearest reachable free spot to the carter's parking place (used when the usual spot cannot be reached)."""
        sx, sy = c.park_xy
        pd = self._pdisk()
        for rad in (0.0, 1.0, 1.8, 2.6, 3.4, 4.2):
            for k in range(12 if rad else 1):
                a = 2 * math.pi * k / 12
                p = (sx + rad * math.cos(a), sy + rad * math.sin(a))
                if self.grid.free_point(p[0], p[1], pd) and \
                        self.grid.plan((c.mob.x, c.mob.y), p, pd, 1.3, 0.3) is not None:
                    return p
        return None

    def carter_proc(self, c):
        c.park_xy = C.PARK_SPOTS.get(c.name, (c.mob.x, c.mob.y))
        spot = c.park_xy
        while True:
            if c.job is None:
                blk, other, threat = self.arbitrate(c)
                if threat is not None:                       # an active carter is coming through: make way
                    yield from self._evade(c, threat)
                    yield
                    continue
                if C.ROAM_WHEN_IDLE and c.roamer:
                    for _ in range(40):
                        gx, gy = random.uniform(-4.5, 5.0), random.uniform(-5.5, 5.5)
                        if self.grid.free_point(gx, gy, self._pdisk()) and all(
                                math.hypot(gx - o.mob.x, gy - o.mob.y) > 2.0
                                for o in self.carters.values() if o is not c):
                            break
                    yield from self.goto(c, (gx, gy), 0.3, abort=lambda: c.job is not None)
                    yield
                    continue
                if math.hypot(c.mob.x - spot[0], c.mob.y - spot[1]) > 0.5:
                    ok = yield from self.goto(c, spot, 0.3, abort=lambda: c.job is not None, dock=True)
                    if not ok and c.job is None:
                        alt = self._alt_park(c)
                        if alt is not None:
                            self.log(f"{c.name}: parking spot unreachable, waiting at ({alt[0]:.1f}, {alt[1]:.1f})")
                            spot = alt
                        else:
                            yield from wait(2.0)
                yield
                continue
            b, f = c.job
            self.log(f"{c.name}: heading to dock {b.tag}")
            yield from self.goto(c, b.W, 0.2, dock=True, must=True)
            yield from self.straight(c, b.dock, C.DOCK_YAW)
            c.parked, b.docked = True, c
            self.log(f"{c.name}: docked at belt {b.tag}")
            while b.docked is c:
                yield
            c.parked = False
            b.carter = None
            # back out to the staging point, then wait for the forklift strip
            yield from self.goto(c, b.W, 0.25, dock=True, must=True)
            while f.owner is not None:
                yield
            f.owner = c
            self.log(f"{c.name}: delivering to {f.name}")
            yield from self.goto(c, f.AP, 0.25, dock=True, must=True)
            yield from self.straight(c, f.CT, f.sgn * math.pi / 2)
            f.carter_free = False
            c.parked, f.carter_at_ct = True, c
            while f.carter_at_ct is c and not f.carter_free:
                yield
            c.parked = False
            yield from self.straight(c, f.AP, tol=0.05)
            f.carter_at_ct = None
            c.job = None
            self.log(f"{c.name}: job done")

    # ------------------------------------------------------------------ arms
    def _carter_box(self, c):
        L, W = C.CARTER_PLATE["size"]
        sx, sy = c.slot_world()
        co, si = math.cos(c.mob.yaw), math.sin(c.mob.yaw)
        pts = [(sx + co * dx - si * dy, sy + si * dx + co * dy) for dx in (-L / 2, L / 2) for dy in (-W / 2, W / 2)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return (np.array([min(xs), min(ys), 0.0]), np.array([max(xs), max(ys), C.CARTER_DECK_H - 0.006]))

    def _plan_arm(self, b, arm, c, pkg, q_home):
        """IK for pick/place with collision checks on every link and on the carried package.
        Tries normal link clearance first, then thin-link clearance.  Returns the four joint solutions or None."""
        H = PKG_H()
        top = pkg.pos() + np.array([0, 0, H / 2 + 0.004])
        sx, sy = c.slot_world()
        place = np.array([sx, sy, C.CARTER_DECK_H + H + 0.004])
        boxes = self.hw.arm_boxes(b.tag) + [self._carter_box(c)]
        names = ["belt", "near rail", "far rail", "stopper", "pedestal", "carter deck"]
        carried = tuple(C.PKG_SIZE)
        why = []
        for rad in (0.05, 0.035):
            for clr in (0.20, 0.30, 0.40):
                lab = ("above pick", "pick", "above place", "place")
                seed, sol, ok = q_home, [], True
                for nm, t in zip(lab, (top + [0, 0, clr], top, place + [0, 0, clr], place)):
                    q, k = arm.ik(np.asarray(t, float), seed)
                    if not k:
                        p, d = arm.tip(q)
                        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(d[2])))))
                        why.append(f"IK misses '{nm}' (clr {clr:.2f}) by {1000 * float(np.linalg.norm(p - np.asarray(t))):.0f} mm / tilt {tilt:.0f} deg")
                        ok = False
                        break
                    if arm.collides(q, boxes, rad):
                        info = arm.collision_info(q, boxes, rad) if hasattr(arm, "collision_info") else None
                        what = f" link {info[0]} vs {names[info[1]] if 0 <= info[1] < len(names) else 'floor'}" if info else ""
                        why.append(f"arm touches an object at '{nm}' (clr {clr:.2f}, r {rad:.3f}){what}")
                        ok = False
                        break
                    sol.append(q)
                    seed = q
                if not ok:
                    continue
                q_ap, q_pk, q_pa, q_pl = sol
                segs = [(q_home, q_ap, None), (q_ap, q_pk, None), (q_pk, q_ap, carried), (q_ap, q_pa, carried),
                        (q_pa, q_pl, carried), (q_pl, q_pa, None), (q_pa, q_home, None)]
                bad = [i for i, (x, y, cr) in enumerate(segs) if not arm.path_free(x, y, boxes, carried=cr, radius=rad)]
                if bad:
                    why.append(f"swept path {bad} hits an object (clr {clr:.2f}, r {rad:.3f})")
                    continue
                if rad < 0.05:
                    self.log(f"arm {b.tag}: plan found with thin-link clearance (r={rad})")
                return sol
        sol = self._plan_arm_branches(arm, boxes, top, place, q_home)
        if sol is not None:
            return sol
        self.log(f"arm {b.tag}: no collision-free plan: " + "; ".join(why[:6]))
        return None

    def _plan_arm_branches(self, arm, boxes, top, place, q_home):
        """Second planner pass: explore the arm's other IK branches for each pose.

        The first pass keeps only the solution nearest the previous pose.  For some arms (UR5)
        that branch drives the upper arm through the pedestal even though another elbow branch
        reaches the same pose cleanly, so for those we search a few branches per pose for a
        continuous, collision-free pick-and-place sequence."""
        carried = tuple(C.PKG_SIZE)
        budget = [400]
        for rad in (0.05, 0.035):
            for clr in (0.20, 0.30, 0.40):
                sets = []
                for t in (top + [0, 0, clr], top, place + [0, 0, clr], place):
                    cand = []
                    for q in arm.ik_candidates(np.asarray(t, float), q_home):
                        if arm.collides(q, boxes, rad):
                            continue
                        cand.append(q)
                        if len(cand) >= 6:
                            break
                    sets.append(cand)
                if any(not s for s in sets):
                    continue
                seq = [q_home]

                def dfs(i):
                    if i == 4:
                        if not arm.path_free(seq[4], seq[3], boxes, radius=rad):
                            return False
                        return arm.path_free(seq[3], q_home, boxes, radius=rad)
                    for q in sets[i]:
                        if budget[0] <= 0:
                            return False
                        budget[0] -= 1
                        cr = carried if i >= 2 else None
                        if not arm.path_free(seq[-1], q, boxes, carried=cr, radius=rad):
                            continue
                        seq.append(q)
                        if dfs(i + 1):
                            return True
                        seq.pop()
                    return False

                if dfs(0):
                    self.log(f"arm {arm.tag}: plan found from an alternate IK branch (r={rad:.3f})")
                    return list(seq[1:])
        return None

    def _move(self, arm, qg, vmax, carry=None):
        q0 = arm.q().copy()
        dq = qg - q0
        T = max(float(np.max(np.abs(dq))) / vmax, 0.25)
        n = int(T / DT)

        def hold(q):
            arm.command(q)
            if carry is not None:
                p, _ = arm.tip(arm.q())
                carry.set_pose(p - np.array([0, 0, PKG_H() + 0.004]) + np.array([0, 0, PKG_H() / 2]), 0.0)

        for i in range(1, n + 1):
            hold(q0 + dq * (0.5 - 0.5 * math.cos(math.pi * i / n)))
            yield
        for _ in range(240):
            if float(np.max(np.abs(arm.q() - qg))) < 0.015:
                return
            hold(qg)
            yield
        err = np.abs(arm.q() - qg)
        self.log(f"arm {arm.name if hasattr(arm, 'name') else '?'}: slow to converge, worst joint #{int(np.argmax(err))} off by {float(err.max()):.3f} rad")

    def arm_proc(self, b, arm):
        while True:
            while not (b.ready and b.docked is not None and b.docked.parked):
                yield
            c, pkg = b.docked, b.pkg
            H = PKG_H()
            q_home = arm.q().copy()
            sol = self._plan_arm(b, arm, c, pkg, q_home) if arm.ok else None
            sx, sy = c.slot_world()
            if sol is not None:
                q_ap, q_pk, q_pa, q_pl = sol
                top = pkg.pos() + np.array([0, 0, H / 2 + 0.004])
                self.log(f"arm {b.tag}: picking")
                yield from self._move(arm, q_ap, 1.3)
                yield from self._move(arm, q_pk, 0.45)
                p, _ = arm.tip(arm.q())
                dist = float(np.linalg.norm(p - top))
                self.log(f"arm {b.tag}: cup {1000 * dist:.0f} mm from package top, vacuum ON")
                pkg.detach_physics()
                yield from wait(0.3)
                yield from self._move(arm, q_ap, 0.5, pkg)
                yield from self._move(arm, q_pa, 1.3, pkg)
                yield from self._move(arm, q_pl, 0.4, pkg)
                yield from wait(0.25)
                c.cargo = pkg
                sx, sy = c.slot_world()
                pkg.set_pose([sx, sy, C.CARTER_DECK_H + H / 2], c.mob.yaw)
                self.log(f"arm {b.tag}: vacuum OFF, package on {c.name}")
                yield from self._move(arm, q_pa, 0.8)
                yield from self._move(arm, q_home, 1.3)
            else:
                self.log(f"arm {b.tag}: using direct-transfer fallback")
                pkg.detach_physics()
                yield from wait(1.0)
                c.cargo = pkg
                pkg.set_pose([sx, sy, C.CARTER_DECK_H + H / 2], c.mob.yaw)
            ps = np.array(c.slot_world())
            good = float(np.linalg.norm(pkg.pos()[:2] - ps)) < 0.06 and abs(pkg.pos()[2] - (C.CARTER_DECK_H + H / 2)) < 0.03
            self.log(f"arm {b.tag}: placement {'OK' if good else 'BAD'}")
            b.docked, b.pkg = None, None            # releases carter, lets belt spawn the next package

    # ------------------------------------------------------------------ forklifts
    def _fapply(self, f):
        self.hw.fork_set(f.name, f.x, f.y, f.lift)
        if f.cargo is not None:
            f.cargo.set_pose([f.x - f.Ctip, f.y, f.lift + f.surf - C.PKG_SKID_H + PKG_H() / 2], f.cargo_yaw)

    def _touch(self, f, bottom):
        """fork lift height at which the fork tops touch the underside of a crate whose skids stand at `bottom`."""
        return bottom + C.PKG_SKID_H - f.surf

    def _fdrive(self, f, tx, ty, v=0.7):
        while True:
            if any(math.hypot(f.x - p["mob"].x, f.y - p["mob"].y) < 1.5 for p in self.people):
                yield                      # never drive the forklift into a person or the pushcart
                continue
            dx, dy = tx - f.x, ty - f.y
            d = math.hypot(dx, dy)
            if d < 1e-3:
                break
            s = min(v * DT, d)
            f.x, f.y = f.x + dx / d * s, f.y + dy / d * s
            self._fapply(f)
            yield

    def _flift(self, f, h, v=0.25):
        h = max(h, 0.0)
        while abs(f.lift - h) > 1e-3:
            f.lift += max(-v * DT, min(v * DT, h - f.lift))
            self._fapply(f)
            yield

    def _slot(self, f, k):
        """(x, y, skid-bottom z) of the k-th crate on this forklift's pallet. Crates stand with their long side along y."""
        L, D, H = C.PKG_SIZE
        px, py, top, sx, sy = self.hw.pallet_info(f.name)
        nxs, nys = max(1, int(sx // (D + 0.04))), max(1, int(sy // (L + 0.04)))
        per = nxs * nys
        layers = 3
        k %= per * layers
        layer, r = divmod(k, per)
        row, col = divmod(r, nxs)
        return (px + (col - (nxs - 1) / 2) * (D + 0.04), py + (row - (nys - 1) / 2) * (L + 0.04), top + layer * H)

    def _in_strip(self, f, c, m=0.5):
        return 5.7 - m < c.mob.x < 12.9 + m and f.fy - 1.0 - m < c.mob.y < f.fy + 1.0 + m

    def fork_proc(self, f):
        H = PKG_H()
        while True:
            c = None
            while True:
                c = f.carter_at_ct
                if c is not None and c.parked and c.cargo is not None:
                    break
                yield
            pkg = c.cargo
            self.log(f"{f.name}: collecting package from {c.name}")
            f.carter_free = False
            px, py = pkg.pos()[0], pkg.pos()[1]
            touch = self._touch(f, C.CARTER_DECK_H)
            yield from self._flift(f, touch - 0.025)              # forks under the crate body, between the skids
            yield from self._fdrive(f, px + f.Ctip, py)
            yield from self._flift(f, touch)                      # forks touch the body
            c.cargo, f.cargo, f.cargo_yaw = None, pkg, c.mob.yaw
            yield from self._flift(f, touch + 0.06)               # crate clear of the load deck
            yield from self._fdrive(f, f.fx, f.fy)
            f.carter_free = True
            yield from self._flift(f, max(C.FORK_CARRY_LIFT, C.PKG_SKID_H - f.surf + 0.10))
            while self._in_strip(f, c) or f.carter_at_ct is not None:
                yield
            f.owner = "F"
            slot = self._slot(f, f.delivered)
            self.log(f"{f.name}: placing on pallet slot {f.delivered}")
            yield from self._flift(f, self._touch(f, slot[2]) + 0.03)
            yield from self._fdrive(f, slot[0] + f.Ctip, slot[1])
            yield from self._flift(f, self._touch(f, slot[2]))     # crate rests on the pallet / crate below
            pkg.set_pose([slot[0], slot[1], slot[2] + H / 2], f.cargo_yaw)
            f.cargo = None
            f.delivered += 1
            yield from self._flift(f, self._touch(f, slot[2]) - 0.05)   # forks free of the crate before backing out
            yield from self._fdrive(f, f.fx, slot[1])
            yield from self._fdrive(f, f.fx, f.fy)
            yield from self._flift(f, 0.0)
            f.owner = None
            if f.belt is not None:
                f.belt.line_busy = False
            self.log(f"{f.name}: delivered total {f.delivered}")
