#!/usr/bin/env python3
"""Warehouse management system (WMS) node.

Issues orders to the cell over ROS 2 and follows each one to the pallet using the cell's event
stream. Runs with the system ROS 2 (Jazzy) Python, separate from the Isaac Sim process.

  out  /warehouse/orders  std_msgs/String  {"id": "ORD-0001", "line": "A", "qty": 1}
       /wms/status        std_msgs/String  JSON, 1 Hz (orders + counts + throughput)
  in   /wms/new_order     std_msgs/String  {"line": "A", "qty": 1}   (manual order, e.g. from the web)
       /wms/config        std_msgs/String  {"auto": true, "period_s": 40}
       /warehouse/events  std_msgs/String  (cell events: release / pick / delivery)
       /warehouse/layout  std_msgs/String  (which forklift serves which line)
       /clock             rosgraph_msgs/Clock (simulation time)

Parameters: auto (bool, true), period_s (float, 45.0: one order per line every period),
            max_open_per_line (int, 2: auto never builds a longer queue than this per line)
"""
import json, re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

STATES = ("queued", "released", "on_carter", "delivered")


class WMS(Node):
    def __init__(self):
        super().__init__("wms")
        self.declare_parameter("auto", True)
        self.declare_parameter("period_s", 45.0)
        self.declare_parameter("max_open_per_line", 2)
        self.auto = bool(self.get_parameter("auto").value)
        self.period = float(self.get_parameter("period_s").value)
        self.max_open = int(self.get_parameter("max_open_per_line").value)
        self.lines = ["A", "B"]
        self.fork_line = {"F1": "A", "F2": "B"}
        self.orders, self.seq = [], 0
        self.t, self.last_auto = 0.0, None
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p_order = self.create_publisher(String, "/warehouse/orders", rel)
        self.p_status = self.create_publisher(String, "/wms/status", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, "/wms/new_order", self.on_new_order, rel)
        self.create_subscription(String, "/wms/config", self.on_config, rel)
        self.create_subscription(String, "/warehouse/events", self.on_event, rel)
        self.create_subscription(String, "/warehouse/layout", self.on_layout, latched)
        self.create_subscription(Clock, "/clock", self.on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"WMS up: auto={self.auto} period={self.period:.0f}s max_open={self.max_open}")

    # ---------------------------------------------------------------- inputs
    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if t + 5.0 < self.t:                       # the cell restarted: its counters are gone too
            self.get_logger().warn("simulation time went backwards - cell restarted, clearing open orders")
            self.orders = [o for o in self.orders if o["state"] == "delivered"]
            self.last_auto = None
        self.t = t

    def on_layout(self, m):
        try:
            lay = json.loads(m.data)
            belts = {b["line"]: b["y"] for b in lay.get("belts", [])}
            for s in lay.get("forklift_strips", []):
                y = (s["y0"] + s["y1"]) / 2
                self.fork_line[s["name"]] = min(belts, key=lambda k: abs(belts[k] - y))
            lines = sorted(belts) or self.lines
            if (lines, self.fork_line) != getattr(self, "_seen_layout", None):   # republished every 10 s
                self._seen_layout = (lines, dict(self.fork_line))
                self.get_logger().info(f"layout: lines {lines}, forklift -> line {self.fork_line}")
            self.lines = lines
        except Exception as e:
            self.get_logger().warn(f"layout not understood: {e}")

    def on_new_order(self, m):
        try:
            d = json.loads(m.data)
            line, qty = str(d.get("line", "")).upper(), int(d.get("qty", 1))
        except Exception:
            line, qty = m.data.strip().upper(), 1
        if line not in self.lines or not 1 <= qty <= 20:
            self.get_logger().warn(f"rejected order request {m.data!r}")
            return
        for _ in range(qty):
            self.issue(line, "manual")

    def on_config(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            self.get_logger().warn(f"bad config {m.data!r}")
            return
        if "auto" in d:
            self.auto = bool(d["auto"])
        if "period_s" in d and float(d["period_s"]) >= 5.0:
            self.period = float(d["period_s"])
        if "max_open_per_line" in d:
            self.max_open = max(1, int(d["max_open_per_line"]))
        self.get_logger().info(f"config: auto={self.auto} period={self.period:.0f}s max_open={self.max_open}")

    def on_event(self, m):
        msg = re.sub(r"^\[\s*[0-9.]+s\]\s*", "", m.data)
        mm = re.match(r"belt (\w): new package on belt", msg)
        if mm:
            self.advance(mm.group(1), "queued", "released")
            return
        mm = re.match(r"arm (\w): vacuum OFF, package on (\w+)", msg)
        if mm:
            self.advance(mm.group(1), "released", "on_carter", carter=mm.group(2))
            return
        mm = re.match(r"(F\d): delivered total", msg)
        if mm:
            line = self.fork_line.get(mm.group(1))
            if line:
                self.advance(line, "on_carter", "delivered", forklift=mm.group(1))

    # ---------------------------------------------------------------- logic
    def issue(self, line, source):
        self.seq += 1
        o = dict(id=f"ORD-{self.seq:04d}", line=line, qty=1, state="queued", source=source,
                 created_t=round(self.t, 1), done_t=None)
        self.orders.append(o)
        self.p_order.publish(String(data=json.dumps(dict(id=o["id"], line=line, qty=1))))
        self.get_logger().info(f"{o['id']} line {line} ({source})")

    def advance(self, line, frm, to, **extra):
        for o in self.orders:
            if o["line"] == line and o["state"] == frm:
                o["state"] = to
                o.update(extra)
                if to == "delivered":
                    o["done_t"] = round(self.t, 1)
                return

    def tick(self):
        if not self.auto or self.t <= 0.0:
            return
        if self.last_auto is not None and self.t - self.last_auto < self.period:
            return
        self.last_auto = self.t
        for line in self.lines:
            open_ = sum(1 for o in self.orders if o["line"] == line and o["state"] != "delivered")
            if open_ < self.max_open:
                self.issue(line, "auto")

    def publish_status(self):
        done = [o for o in self.orders if o["state"] == "delivered"]
        recent = [o for o in done if o["done_t"] is not None and self.t - o["done_t"] <= 1800.0]
        span = min(max(self.t, 1.0), 1800.0)
        lead = [o["done_t"] - o["created_t"] for o in done[-20:] if o["done_t"] is not None]
        st = dict(auto=self.auto, period_s=self.period, max_open_per_line=self.max_open, t=round(self.t, 1),
                  orders=self.orders[-40:],
                  counts=dict(queued=sum(o["state"] == "queued" for o in self.orders),
                              in_progress=sum(o["state"] in ("released", "on_carter") for o in self.orders),
                              delivered=len(done)),
                  throughput_per_hour=round(len(recent) * 3600.0 / span, 1),
                  mean_lead_time_s=round(sum(lead) / len(lead), 1) if lead else None)
        if len(self.orders) > 500:
            self.orders = [o for o in self.orders if o["state"] != "delivered"] + done[-200:]
        self.p_status.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    n = WMS()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
