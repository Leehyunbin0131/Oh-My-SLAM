"""Spin one node until SIGINT/SIGTERM or external shutdown, then tear down in order."""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def spin_node(node_factory, *, args=None, before_destroy=None, spin_timeout=.1):
    """Run node_factory() on a single-threaded executor.

    rclpy's signal handlers are disabled, so SIGINT and SIGTERM only request
    the spin loop to end. The context therefore stays valid while
    before_destroy(node) runs, before the executor, node and context are shut
    down. The original Python signal handlers are always restored. Exceptions
    from node_factory or before_destroy propagate after that teardown.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous = {}
    executor = node = None
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        executor = SingleThreadedExecutor()
        node = node_factory()
        executor.add_node(node)
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=spin_timeout)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if node is not None and before_destroy is not None:
                before_destroy(node)
        finally:
            try:
                if executor is not None:
                    executor.shutdown()
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    rclpy.try_shutdown()
                finally:
                    for sig, handler in previous.items():
                        signal.signal(sig, handler)
