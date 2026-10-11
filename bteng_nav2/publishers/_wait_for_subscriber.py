"""Shared publish-when-matched behaviour for the publisher nodes.

A publisher created inside tick() has not been matched to any subscriber yet:
DDS discovery takes tens to hundreds of milliseconds, and a message published
before the match is dropped on the floor with no error. Both publisher nodes
here send exactly one message that something downstream must receive -- an
initial pose AMCL never sees is the difference between a robot that localizes
and one that does not -- so publishing on the first tick and reporting SUCCESS
is reporting a delivery that did not happen.

These nodes therefore create the publisher, then report RUNNING until a
subscriber is connected (or match_timeout elapses), publish once, and only then
report SUCCESS. A transport with no subscriber-count support, or a timeout,
still publishes -- best effort beats refusing to send.
"""
from __future__ import annotations

import time

from bteng import NodeStatus


class WaitForSubscriberMixin:
    """Publish once a subscriber is matched. Used by the publisher nodes."""

    #: Seconds to wait for a subscriber before publishing anyway. 0 publishes
    #: on the first tick, which is the pre-0.2.1 behaviour.
    match_timeout: float = 2.0

    def _publish_when_matched(self, msg_type, message) -> NodeStatus:
        if not hasattr(self, "_pub"):
            self._pub = self.create_publisher(msg_type, self.topic, 10)
            self._match_deadline = time.monotonic() + self.match_timeout
            self._published = False

        if self._published:
            return NodeStatus.SUCCESS

        if self.match_timeout > 0 and not self._has_subscriber():
            if time.monotonic() < self._match_deadline:
                self.set_feedback_message(f"waiting for a subscriber on {self.topic}")
                return NodeStatus.RUNNING
            self.set_feedback_message(
                f"no subscriber on {self.topic} after {self.match_timeout}s — publishing anyway"
            )

        self._pub.publish(message)
        self._published = True
        return NodeStatus.SUCCESS

    def _has_subscriber(self) -> bool:
        probe = getattr(self._pub, "get_subscription_count", None)
        if not callable(probe):
            return True  # a transport or test double that cannot tell us: assume matched
        try:
            return probe() > 0
        except Exception:
            return True

    def _on_reset(self) -> None:
        """Republish on the next activation rather than replaying SUCCESS."""
        self._published = False
        if hasattr(self, "_pub"):
            self._match_deadline = time.monotonic() + self.match_timeout
