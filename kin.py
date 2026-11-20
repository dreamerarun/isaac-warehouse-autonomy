"""Generic serial-arm kinematics (FK + damped-least-squares IK) built from joint frames.
Pure numpy so it can be tested off-line. Chain model:  T_body1 = T_body0 @ F0 @ Rot(axis,q) @ F1inv."""
import math
import numpy as np


def rot_axis(a, q):
    x, y, z = a
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def T_of(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def quat_R(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Joint:
    def __init__(self, name, kind, F0, F1inv, axis=(0, 0, 1), lo=-math.pi, hi=math.pi):
        self.name, self.kind, self.F0, self.F1inv = name, kind, F0, F1inv
        self.axis = np.asarray(axis, float)
        self.lo, self.hi = lo, hi


class Chain:
    def __init__(self, T_world_base, joints, tool_dir, tool_len):
        self.T0 = T_world_base
        self.joints = joints
        self.act = [j for j in joints if j.kind == "rev"]
        self.n = len(self.act)
        self.lo = np.array([j.lo for j in self.act])
        self.hi = np.array([j.hi for j in self.act])
        self.tool_dir = np.asarray(tool_dir, float)
        self.tool_len = tool_len

    def fk(self, q, upto=None):
        T, k = self.T0.copy(), 0
        for j in self.joints:
            T = T @ j.F0
            if j.kind == "rev":
                T = T @ T_of(rot_axis(j.axis, q[k]), np.zeros(3))
                k += 1
            T = T @ j.F1inv
        return T

    def frames(self, q):
        """World positions of every joint frame origin followed by the tool tip (list of 3-vectors)."""
        T, k, pts = self.T0.copy(), 0, [self.T0[:3, 3].copy()]
        for j in self.joints:
            T = T @ j.F0
            if j.kind == "rev":
                T = T @ T_of(rot_axis(j.axis, q[k]), np.zeros(3))
                k += 1
            T = T @ j.F1inv
            pts.append(T[:3, 3].copy())
        tip, _ = self.tip(q)
        pts.append(tip)
        return pts

    def collides(self, q, boxes, radius=0.05, skip_first=1, tool_radius=0.035):
        """True if any link segment (capsule) hits an axis-aligned box (min3, max3).
        skip_first: ignore the first N segments (base column)."""
        pts = self.frames(q)
        segs = list(zip(pts[:-1], pts[1:]))
        for i, (a, b) in enumerate(segs):
            if i < skip_first:
                continue
            r = tool_radius if i == len(segs) - 1 else radius
            n = max(int(np.linalg.norm(b - a) / 0.02), 1)
            for k in range(n + 1):
                p = a + (b - a) * (k / n)
                if p[2] < r * 0.5:                      # floor
                    return True
                for lo, hi in boxes:
                    if (p[0] > lo[0] - r and p[0] < hi[0] + r and p[1] > lo[1] - r and p[1] < hi[1] + r
                            and p[2] > lo[2] - r and p[2] < hi[2] + r):
                        return True
        return False

    def collision_info(self, q, boxes, radius=0.05, skip_first=1, tool_radius=0.035):
        """(link index, box index) of the first contact, or None."""
        pts = self.frames(q)
        segs = list(zip(pts[:-1], pts[1:]))
        for i, (a, b) in enumerate(segs):
            if i < skip_first:
                continue
            r = tool_radius if i == len(segs) - 1 else radius
            n = max(int(np.linalg.norm(b - a) / 0.02), 1)
            for kk in range(n + 1):
                p = a + (b - a) * (kk / n)
                if p[2] < r * 0.5:
                    return (i, -1)
                for j, (lo, hi) in enumerate(boxes):
                    if (lo[0] - r < p[0] < hi[0] + r and lo[1] - r < p[1] < hi[1] + r and lo[2] - r < p[2] < hi[2] + r):
                        return (i, j)
        return None

    def carried_hits(self, q, boxes, size, drop=0.004):
        """AABB of a package hanging under the tip (yaw 0, size=(L, D, H)) against boxes."""
        p, _ = self.tip(q)
        lo = np.array([p[0] - size[0] / 2, p[1] - size[1] / 2, p[2] - drop - size[2]])
        hi = np.array([p[0] + size[0] / 2, p[1] + size[1] / 2, p[2] - drop])
        for b_lo, b_hi in boxes:
            if np.all(lo < b_hi) and np.all(hi > b_lo):
                return True
        return False

    def path_free(self, q0, q1, boxes, n=24, carried=None, **kw):
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        for k in range(n + 1):
            q = q0 + (q1 - q0) * (k / n)
            if self.collides(q, boxes, **kw) or (carried is not None and self.carried_hits(q, boxes, carried)):
                return False
        return True

    def tip(self, q):
        T = self.fk(q)
        return T[:3, 3] + T[:3, :3] @ (self.tool_dir * self.tool_len), T[:3, :3] @ self.tool_dir

    def _res(self, q, target, w):
        p, d = self.tip(q)
        return np.concatenate([p - target, w * np.cross(d, [0.0, 0.0, -1.0])])

    def ik(self, target, seed, tol=2e-3, iters=250, w=0.25, tries=12, rng=None):
        """Tip at target with tool axis pointing straight down. Returns (q, ok)."""
        rng = rng or np.random.default_rng(1)
        best, best_err = None, 1e9
        seeds = [np.array(seed, float)] + [np.clip(np.array(seed) + rng.normal(0, 0.6, self.n),
                                                   self.lo, self.hi) for _ in range(tries)]
        for s in seeds:
            q = np.clip(s, self.lo, self.hi)
            for _ in range(iters):
                r = self._res(q, target, w)
                if np.linalg.norm(r[:3]) < tol and np.linalg.norm(r[3:]) / w < 0.02:
                    break
                J = np.zeros((6, self.n))
                for i in range(self.n):
                    dq = q.copy()
                    dq[i] += 1e-5
                    J[:, i] = (self._res(dq, target, w) - r) / 1e-5
                step = -J.T @ np.linalg.solve(J @ J.T + 0.02 ** 2 * np.eye(6), r)
                nrm = np.linalg.norm(step)
                if nrm > 0.25:
                    step *= 0.25 / nrm
                q = np.clip(q + step, self.lo, self.hi)
            r = self._res(q, target, w)
            err = np.linalg.norm(r[:3]) + np.linalg.norm(r[3:]) / w * 0.05
            ok = np.linalg.norm(r[:3]) < tol * 2 and np.linalg.norm(r[3:]) / w < 0.05
            score = err + 0.01 * np.linalg.norm(q - seed)
            if ok and score < best_err:
                best, best_err = q.copy(), score
                break  # first converged solution nearest the seed ordering
            if best is None and err < 1e8:
                pass
        return (best, True) if best is not None else (np.array(seed, float), False)

    def ik_candidates(self, target, seed, n=12, spread=1.0, rng=None, **kw):
        """Up to n distinct IK solutions for the same pose, nearest seed first.

        A 6-DOF arm has several elbow/wrist branches that all satisfy the tool pose; the
        nearest one is not always usable, so planners need alternatives to test for collisions."""
        rng = rng or np.random.default_rng(7)
        seed = np.asarray(seed, float)
        seeds = [seed] + [np.clip(seed + rng.normal(0, spread, self.n), self.lo, self.hi)
                          for _ in range(2 * n)]
        out = []
        for s in seeds:
            q, ok = self.ik(target, s, tries=0, **kw)
            if not ok:
                continue
            if all(np.max(np.abs((q - a + np.pi) % (2 * np.pi) - np.pi)) > 0.35 for a in out):
                out.append(q)
                if len(out) >= n:
                    break
        return out


# ---- synthetic arms for off-line tests (not used on the real robot) -------------
def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return T_of(np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), np.zeros(3))


def _tx(a): return T_of(np.eye(3), [a, 0, 0])
def _tz(d): return T_of(np.eye(3), [0, 0, d])


def synthetic_ur5(T_world_base):
    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
    a = [0, -0.425, -0.39225, 0, 0, 0]
    al = [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0]
    js = [Joint(f"ur{i}", "rev", np.eye(4), _tz(d[i]) @ _tx(a[i]) @ _rx(al[i]), lo=-2 * math.pi, hi=2 * math.pi)
          for i in range(6)]
    return Chain(T_world_base, js, (0, 0, 1), 0.105)


def synthetic_franka(T_world_base):
    a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
    d = [0.333, 0, 0.316, 0, 0.384, 0, 0]
    al = [0, -math.pi / 2, math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, math.pi / 2]
    lim = [(-2.89, 2.89), (-1.76, 1.76), (-2.89, 2.89), (-3.07, -0.07), (-2.89, 2.89), (-0.02, 3.75), (-2.89, 2.89)]
    js = [Joint(f"fr{i}", "rev", _rx(al[i]) @ _tx(a[i]), _tz(d[i]), lo=lim[i][0], hi=lim[i][1]) for i in range(7)]
    js.append(Joint("flange", "fixed", np.eye(4), _tz(0.107)))
    return Chain(T_world_base, js, (0, 0, 1), 0.065 + 0.105)
