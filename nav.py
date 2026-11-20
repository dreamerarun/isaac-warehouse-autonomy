"""Occupancy-grid A* planner + differential-drive (unicycle) vehicle model. Pure Python/numpy."""
import heapq, math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Grid:
    def __init__(self, x0, x1, y0, y1, res=0.1):
        self.res, self.x0, self.y0 = res, x0, y0
        self.nx, self.ny = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
        xs = x0 + (np.arange(self.nx) + 0.5) * res
        ys = y0 + (np.arange(self.ny) + 0.5) * res
        self.X, self.Y = np.meshgrid(xs, ys, indexing="ij")
        self.soft = np.zeros((self.nx, self.ny), bool)
        self.hard = np.zeros((self.nx, self.ny), bool)
        self.raw = np.zeros((self.nx, self.ny), bool)

    def build(self, rects, r_soft, r_hard, r_raw=0.40):
        self.soft[:] = False
        self.hard[:] = False
        self.raw[:] = False
        for (a, b, c, d) in rects:
            for arr, r in ((self.soft, r_soft), (self.hard, r_hard), (self.raw, r_raw)):
                arr |= (self.X >= a - r) & (self.X <= b + r) & (self.Y >= c - r) & (self.Y <= d + r)

    def cell(self, x, y):
        return (min(max(int((x - self.x0) / self.res), 0), self.nx - 1),
                min(max(int((y - self.y0) / self.res), 0), self.ny - 1))

    def world(self, i, j):
        return (self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res)

    def _los(self, blocked, p, q):
        n = max(int(math.hypot(q[0] - p[0], q[1] - p[1]) / (self.res * 0.5)), 1)
        for k in range(n + 1):
            t = k / n
            i, j = self.cell(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
            if blocked[i, j]:
                return False
        return True

    def plan(self, start, goal, disks=(), ex_r=1.3, ex_goal=None):
        """start/goal (x,y). Near start/goal only the 'hard' (thin) inflation applies.
        disks: extra dynamic obstacles (x, y, r). Returns list of waypoints or None."""
        exg = ex_r if ex_goal is None else ex_goal
        in_soft = bool(self.soft[self.cell(*start)])                                   # only relax when we really start inside a keep-out zone
        exs = ((self.X - start[0]) ** 2 + (self.Y - start[1]) ** 2 < ex_r ** 2) & in_soft        # leaving a dock / parking spot
        exd = (self.X - goal[0]) ** 2 + (self.Y - goal[1]) ** 2 < exg ** 2           # arriving at one
        blocked = np.where(exs, self.raw, np.where(exd, self.raw if exg > 0.5 else self.hard, self.soft))
        for (dx, dy, dr) in disks:
            blocked = blocked | (((self.X - dx) ** 2 + (self.Y - dy) ** 2) < dr ** 2)
        s, g = self.cell(*start), self.cell(*goal)
        blocked = blocked.copy()
        blocked[s] = False
        if blocked[g]:
            g = self._nearest_free(blocked, g)
            if g is None:
                return None
        openq = [(0.0, s)]
        gs, par = {s: 0.0}, {}
        nb = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for di, dj, w in nb:
                n = (cur[0] + di, cur[1] + dj)
                if not (0 <= n[0] < self.nx and 0 <= n[1] < self.ny) or blocked[n]:
                    continue
                if di and dj and (blocked[cur[0] + di, cur[1]] or blocked[cur[0], cur[1] + dj]):
                    continue
                ng = gs[cur] + w
                if ng < gs.get(n, 1e18):
                    gs[n], par[n] = ng, cur
                    h = math.hypot(n[0] - g[0], n[1] - g[1])
                    heapq.heappush(openq, (ng + h, n))
        else:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(par[cells[-1]])
        cells.reverse()
        pts = [start] + [self.world(*c) for c in cells[1:]]
        # line-of-sight shortcutting
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._los(blocked, pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        out = out[1:]
        if out:
            out[-1] = goal if not self._near_blocked_goal(blocked, goal) else out[-1]
        return out

    def _near_blocked_goal(self, blocked, goal):
        return bool(blocked[self.cell(*goal)])

    def _nearest_free(self, blocked, c, rmax=12):
        for r in range(1, rmax):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    n = (c[0] + di, c[1] + dj)
                    if 0 <= n[0] < self.nx and 0 <= n[1] < self.ny and not blocked[n]:
                        return n
        return None

    def free_point(self, x, y, disks=()):
        i, j = self.cell(x, y)
        if self.soft[i, j]:
            return False
        return all(math.hypot(x - dx, y - dy) > dr for dx, dy, dr in disks)


class Mobile:
    """Differential-drive kinematic model: turn in place when badly aligned, else drive + steer."""

    def __init__(self, x, y, yaw, vmax=0.9, wmax=1.6, amax=0.8):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.vmax, self.wmax, self.amax = vmax, wmax, amax

    def pos(self):
        return np.array([self.x, self.y])

    def _integrate(self, v_cmd, w, dt):
        self.v += max(-self.amax * dt, min(self.amax * dt, v_cmd - self.v))
        self.yaw = wrap(self.yaw + w * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def track(self, wp, dt, move=True, tol=0.15):
        dx, dy = wp[0] - self.x, wp[1] - self.y
        d = math.hypot(dx, dy)
        if d < tol:
            self._integrate(0.0, 0.0, dt)
            return True
        err = wrap(math.atan2(dy, dx) - self.yaw)
        w = max(-self.wmax, min(self.wmax, 3.0 * err))
        v = 0.0
        if move and abs(err) < 0.5:
            v = min(self.vmax, 1.2 * d) * max(0.0, math.cos(err))
        self._integrate(v, w, dt)
        return False

    def drive_straight(self, wp, dt, tol=0.03, move=True):
        """Go to wp holding current heading axis (used for docking funnels)."""
        dx, dy = wp[0] - self.x, wp[1] - self.y
        d = math.hypot(dx, dy)
        if d < tol:
            self._integrate(0.0, 0.0, dt)
            return True
        err = wrap(math.atan2(dy, dx) - self.yaw)
        w = max(-self.wmax, min(self.wmax, 3.0 * err))
        v = min(0.35, 1.0 * d) * max(0.0, math.cos(err)) if move else 0.0
        self._integrate(v, w, dt)
        return False

    def turn_to(self, yaw_des, dt, tol=0.03):
        err = wrap(yaw_des - self.yaw)
        if abs(err) < tol and abs(self.v) < 0.02:
            return True
        w = max(-self.wmax, min(self.wmax, 3.0 * err))
        self._integrate(0.0, w, dt)
        return abs(err) < tol and abs(self.v) < 0.02
