"""Decide when a request may start, independently of the ROS clock."""


def should_replan(now, last_attempt, pending_reason, min_interval, *,
                  have_path=False, period=0.0, urgent=False):
    """Start new goals and urgent requests before applying the retry interval."""
    if pending_reason == 'new goal' or (pending_reason and urgent):
        return pending_reason
    elapsed = float('inf')
    if last_attempt is not None and now >= last_attempt:
        elapsed = now - last_attempt
    if elapsed < min_interval:
        return None
    if pending_reason:
        return pending_reason
    if have_path and period > 0.0 and elapsed >= period:
        return 'periodic'
    return None
