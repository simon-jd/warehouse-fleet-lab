"""One bounded round of read-only ROS readiness queries; safe to repeat."""

import time

import rclpy


def readiness_round(node, queries, timeout):
    if (
        not queries
        or timeout <= 0
        or not all(client.service_is_ready() for client, _, _ in queries)
    ):
        return False
    deadline = time.monotonic() + timeout
    pending = []
    try:
        for client, request, predicate in queries:
            pending.append((client, client.call_async(request), predicate))
        while not all(future.done() for _, future, _ in pending):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                node.get_logger().warning(
                    "Readiness response deadline expired; pending calls discarded"
                )
                return False
            rclpy.spin_once(node, timeout_sec=min(0.03, remaining))
        return all(
            not future.cancelled() and predicate(future.result())
            for _, future, predicate in pending
        )
    finally:
        for client, future, _ in pending:
            client.remove_pending_request(future)
            if not future.done():
                future.cancel()
