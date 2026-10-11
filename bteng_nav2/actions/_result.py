"""Get at an action's Result message, whatever ``action_result`` hands back.

``RosActionClientMixin.action_result`` is documented as "Result message from the
last completed action", but it holds what ``get_result_async()`` resolved to,
which is the **GetResult response**: a wrapper carrying ``status`` and the actual
``result``.  The mixin's own code says so -- it reads ``action_result.status``
against ``GoalStatus.STATUS_SUCCEEDED``, and a Result message has no ``status``.

Reading a payload field straight off it therefore raises on a real graph::

    'ComputePathToPose_GetResult_Response' object has no attribute 'path'

and nothing offline sees it, because a ``MagicMock`` answers to every attribute
name.  Four nodes here did exactly that until the first live run; two others
happened to be right, because the field they wanted was called ``result``.

Unwrapping instead of pinning the shape keeps this working against both, so a
change on either side of the boundary does not break the nodes again.
"""
from __future__ import annotations


def action_result_of(node):
    """The Result message from *node*'s completed action, or None.

    Accepts the GetResult response, a bare Result message, or None.
    """
    raw = node.action_result
    if raw is None:
        return None
    inner = getattr(raw, "result", None)
    return raw if inner is None else inner
