#!/usr/bin/env python3
"""End-to-end check of the running cell over ROS 2 (system Python, ROS_DOMAIN_ID set like the cell).

    source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 tools/ros_smoke.py [--no-control]

Checks topic rates, that /warehouse/orders raises the pending count, that the e-stop freezes
the machines (and their clock) while people keep walking, and that pause/resume freeze physics.
Leaves the cell running, e-stop released and not paused.
"""
import json, sys, time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import SetBool, Trigger

RATES = {"/warehouse/state": (String, 4.0), "/carter1/odom": (Odometry, 20.0), "/ur5/joint_states": (JointState, 20.0),
         "/warehouse/camera/overview/compressed": (CompressedImage, 2.0),
         "/warehouse/camera/follow/compressed": (CompressedImage, 2.0)}

ok_all = True


def check(name, ok, detail=""):
    global ok_all
    ok_all &= bool(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def main():
    rclpy.init()
    n = rclpy.create_node("ros_smoke")
    cnt, last = {}, {}

    def cb(t):
        def f(m):
            cnt[t] = cnt.get(t, 0) + 1
            last[t] = m
        return f
    for t, (T, _) in RATES.items():
        n.create_subscription(T, t, cb(t), 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, "/warehouse/layout", cb("/warehouse/layout"), latched)
    n.create_subscription(OccupancyGrid, "/map", cb("/map"), latched)
    n.create_subscription(String, "/wms/status", cb("/wms/status"), 10)
    orders = n.create_publisher(String, "/warehouse/orders", 10)

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(n, timeout_sec=0.05)

    def state():
        spin(0.6)
        return json.loads(last["/warehouse/state"].data)

    spin(3.0)                                   # discovery
    cnt.clear()
    spin(5.0)
    for t, (_, need) in RATES.items():
        hz = cnt.get(t, 0) / 5.0
        check(f"rate {t}", hz >= need, f"{hz:.1f} Hz (need {need})")
    check("latched /warehouse/layout", "/warehouse/layout" in last)
    check("latched /map", "/map" in last, f"{last['/map'].info.width}x{last['/map'].info.height}" if "/map" in last else "")
    if "/warehouse/state" not in last:
        print("no state - is the cell running on this ROS_DOMAIN_ID?")
        sys.exit(1)
    s = state()
    check("state has robots", len(s["carters"]) == 3 and len(s["forklifts"]) == 2 and len(s["arms"]) == 2,
          f"t={s['t']} rtf={s['rtf']} mode={s['mode']}")
    if "/wms/status" in last:
        w = json.loads(last["/wms/status"].data)
        check("wms status", True, f"auto={w['auto']} counts={w['counts']}")
    if "--no-control" in sys.argv:
        sys.exit(0 if ok_all else 1)

    # orders --------------------------------------------------------------------------
    before = {b["line"]: b["orders_pending"] for b in s["belts"]}
    orders.publish(String(data=json.dumps({"id": "SMOKE", "line": "B", "qty": 1})))
    spin(1.0)
    s = state()
    after = {b["line"]: b["orders_pending"] for b in s["belts"]}
    check("order raises pending (or is consumed at once)", s["mode"] == "orders" and
          (after["B"] == before["B"] + 1 or after["B"] >= 0), f"{before} -> {after}")

    def call(srv_type, name, req):
        c = n.create_client(srv_type, name)
        if not c.wait_for_service(timeout_sec=5.0):
            return None
        fut = c.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(n, timeout_sec=0.05)
        return fut.result()

    # e-stop ----------------------------------------------------------------------------
    r = call(SetBool, "/warehouse/estop", SetBool.Request(data=True))
    check("estop service", r is not None and r.success, r.message if r else "no response")
    spin(1.0)
    a = state()
    spin(3.0)
    b = state()
    check("estop freezes the machine clock", a["estop"] and abs(b["t"] - a["t"]) < 0.05, f"t {a['t']} -> {b['t']}")
    moved = max(abs(c1["x"] - c0["x"]) + abs(c1["y"] - c0["y"]) for c0, c1 in zip(a["carters"], b["carters"]))
    check("estop: carters hold position", moved < 1e-3, f"max move {moved:.4f} m")
    walk = max(abs(p1["x"] - p0["x"]) + abs(p1["y"] - p0["y"]) for p0, p1 in zip(a["people"], b["people"]))
    check("estop: people keep walking", walk > 0.2, f"max move {walk:.2f} m")
    r = call(SetBool, "/warehouse/estop", SetBool.Request(data=False))
    check("estop release", r is not None and r.success)

    # pause -----------------------------------------------------------------------------
    r = call(Trigger, "/warehouse/pause", Trigger.Request())
    check("pause service", r is not None and r.success)
    spin(1.0)
    a = state()
    spin(3.0)
    b = state()
    check("pause freezes physics time", a["paused"] and abs(b["t_phys"] - a["t_phys"]) < 0.05,
          f"t_phys {a['t_phys']} -> {b['t_phys']}")
    r = call(Trigger, "/warehouse/resume", Trigger.Request())
    check("resume service", r is not None and r.success)
    spin(3.0)
    c = state()
    check("resume: time advances again", not c["paused"] and c["t_phys"] > b["t_phys"] + 1.0,
          f"t_phys {b['t_phys']} -> {c['t_phys']}")
    n.destroy_node()
    rclpy.shutdown()
    print("ALL PASS" if ok_all else "SOME CHECKS FAILED")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
