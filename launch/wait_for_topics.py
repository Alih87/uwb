#!/usr/bin/env python3
import argparse
import sys
import time
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait until one or more ROS 2 topics are stably publishing."
    )
    parser.add_argument("--stage-name", required=True, help="Human-readable stage name")
    parser.add_argument(
        "--spec",
        action="append",
        required=True,
        help=(
            "Topic spec in the form: topic|msg_type|min_msgs|max_age_sec. "
            "Example: /imu/data|sensor_msgs/msg/Imu|10|1.0"
        ),
    )
    parser.add_argument(
        "--stable-for",
        type=float,
        default=3.0,
        help="How long all topics must remain healthy before success.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Overall timeout in seconds. Use 0 for no timeout.",
    )
    parser.add_argument(
        "--status-every",
        type=float,
        default=5.0,
        help="How often to print status lines in seconds.",
    )
    return parser.parse_args()


def parse_spec(spec: str) -> Dict[str, Any]:
    parts = spec.split("|")
    if len(parts) != 4:
        raise ValueError(
            f"Invalid --spec '{spec}'. Expected topic|msg_type|min_msgs|max_age_sec"
        )
    topic, msg_type_str, min_msgs_str, max_age_str = parts
    msg_type = get_message(msg_type_str)
    return {
        "topic": topic,
        "msg_type": msg_type,
        "msg_type_str": msg_type_str,
        "min_msgs": int(min_msgs_str),
        "max_age": float(max_age_str),
        "count": 0,
        "last_time": None,
    }


class TopicWaiter(Node):
    def __init__(self, stage_name: str, specs: List[Dict[str, Any]]) -> None:
        safe_stage = "".join(
            c if (c.isalnum() or c == "_") else "_" for c in stage_name.lower().replace(" ", "_")
        )
        super().__init__(f"wait_for_topics_{safe_stage}")
        self.stage_name = stage_name
        self.specs = specs
        self._subs = []
        for spec in self.specs:
            topic = spec["topic"]
            msg_type = spec["msg_type"]

            def cb(_msg, s=spec):
                s["count"] += 1
                s["last_time"] = time.monotonic()

            self._subs.append(self.create_subscription(msg_type, topic, cb, 10))

    def topic_ok(self, spec: Dict[str, Any], now: float) -> bool:
        return (
            spec["count"] >= spec["min_msgs"]
            and spec["last_time"] is not None
            and (now - spec["last_time"]) <= spec["max_age"]
        )

    def status_string(self, now: float) -> str:
        parts = []
        for spec in self.specs:
            last_age = None if spec["last_time"] is None else round(now - spec["last_time"], 2)
            ok = self.topic_ok(spec, now)
            parts.append(
                f"{spec['topic']}: ok={ok}, count={spec['count']}, last_age={last_age}"
            )
        return " | ".join(parts)


def main() -> int:
    args = parse_args()
    try:
        specs = [parse_spec(s) for s in args.spec]
    except Exception as exc:
        print(f"[wait_for_topics] spec parse error: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = TopicWaiter(args.stage_name, specs)

    start_time = time.monotonic()
    stable_since = None
    last_status = 0.0

    node.get_logger().info(f"[{args.stage_name}] waiting for stable topics...")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()

            all_ok = all(node.topic_ok(spec, now) for spec in specs)
            if all_ok:
                if stable_since is None:
                    stable_since = now
                    node.get_logger().info(
                        f"[{args.stage_name}] all topics healthy, starting stability timer ({args.stable_for:.1f}s)..."
                    )
                elif (now - stable_since) >= args.stable_for:
                    node.get_logger().info(f"[{args.stage_name}] success: topics are stable.")
                    return 0
            else:
                stable_since = None

            if (now - last_status) >= args.status_every:
                node.get_logger().info(f"[{args.stage_name}] {node.status_string(now)}")
                last_status = now

            if args.timeout > 0.0 and (now - start_time) >= args.timeout:
                node.get_logger().error(
                    f"[{args.stage_name}] timeout waiting for stable topics. {node.status_string(now)}"
                )
                return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1


if __name__ == "__main__":
    sys.exit(main())
