"""Check the priority and timing of replan triggers."""

from luna_guidance.replan_policy import should_replan


def test_new_goal_bypasses_the_replan_interval():
    assert should_replan(1.1, 1.0, 'new goal', 0.5) == 'new goal'


def test_first_blocked_replan_bypasses_the_interval():
    assert should_replan(1.1, 1.0, 'path blocked', 0.5,
                         urgent=True) == 'path blocked'


def test_required_replan_retry_waits_until_the_interval_expires():
    assert should_replan(1.25, 1.0, 'path blocked', 0.5) is None
    assert should_replan(1.5, 1.0, 'path blocked', 0.5) == 'path blocked'


def test_urgency_without_a_request_keeps_periodic_work_rate_limited():
    assert should_replan(1.1, 1.0, None, 0.5,
                         have_path=True, period=0.1, urgent=True) is None


def test_periodic_replan_requires_an_active_path_and_does_not_override_blocked():
    assert should_replan(2.0, 1.0, None, 0.5, have_path=True) is None
    assert should_replan(2.0, 1.0, None, 0.5, period=1.0) is None
    assert should_replan(2.0, 1.0, None, 0.5, have_path=True, period=1.0) == 'periodic'
    assert should_replan(2.0, 1.0, 'path blocked', 0.5,
                         have_path=True, period=1.0) == 'path blocked'
