from luna_sim.actuators.timed_action import (CANCELED, SUCCEEDED, TIMEOUT, TimedAction,
                                             make_rng)


def make(
    duration=20.0, jitter=3.0, probability=0.0, reason='stall', seed=0, name='mock_excavation',
):
    return TimedAction(duration, jitter, probability, reason, make_rng(seed, name))


def test_duration_is_base_plus_jitter():
    action = make()
    for _ in range(200):
        plan = action.start()
        assert 20.0 <= plan.duration_s <= 23.0
        assert plan.outcome == SUCCEEDED
        assert plan.end_s == plan.duration_s


def test_zero_jitter_is_exact():
    plan = make(duration=10.0, jitter=0.0).start()
    assert plan.duration_s == 10.0


def test_running_until_duration_then_success():
    plan = make(jitter=0.0).start()
    assert plan.check(0.0) is None
    assert plan.check(19.9) is None
    assert plan.check(20.0) == SUCCEEDED
    assert plan.check(25.0) == SUCCEEDED


def test_cancel_wins_at_any_time():
    plan = make(jitter=0.0).start()
    assert plan.check(1.0, cancel_requested=True) == CANCELED
    assert plan.check(30.0, cancel_requested=True) == CANCELED


def test_timeout_when_sampled_duration_is_longer():
    plan = make(duration=20.0, jitter=0.0).start(max_duration_s=5.0)
    assert plan.end_s == 5.0
    assert plan.check(4.9) is None
    assert plan.check(5.0) == TIMEOUT


def test_no_timeout_when_limit_is_zero_or_long_enough():
    assert make(jitter=0.0).start(max_duration_s=0.0).outcome == SUCCEEDED
    assert make(jitter=0.0).start(max_duration_s=20.0).outcome == SUCCEEDED
    assert make(jitter=0.0).start(max_duration_s=60.0).outcome == SUCCEEDED


def test_failures_never_happen_at_zero_probability():
    action = make(probability=0.0)
    assert all(action.start().outcome == SUCCEEDED for _ in range(500))


def test_failures_always_happen_at_probability_one():
    action = make(probability=1.0, reason='jam')
    for _ in range(100):
        plan = action.start()
        assert plan.outcome == 'jam'
        assert 0.2 * plan.duration_s <= plan.end_s <= 0.8 * plan.duration_s


def test_failure_rate_is_about_the_probability():
    action = make(probability=0.3)
    failures = sum(action.start().outcome != SUCCEEDED for _ in range(2000))
    assert 500 < failures < 700


def test_timeout_beats_a_later_failure():
    action = make(duration=20.0, jitter=0.0, probability=1.0)
    plan = action.start(max_duration_s=1.0)
    assert plan.outcome == TIMEOUT
    assert plan.end_s == 1.0


def test_failure_beats_a_later_timeout():
    action = make(duration=20.0, jitter=0.0, probability=1.0, reason='stall')
    plan = action.start(max_duration_s=19.0)
    assert plan.outcome == 'stall'
    assert plan.end_s <= 16.0


def test_same_seed_same_sequence():
    a = make(seed=7, probability=0.5)
    b = make(seed=7, probability=0.5)
    assert [a.start() for _ in range(20)] == [b.start() for _ in range(20)]


def test_different_seed_different_sequence():
    a = [make(seed=1).start().duration_s for _ in range(5)]
    b = [make(seed=2).start().duration_s for _ in range(5)]
    assert a != b


def test_each_node_gets_its_own_stream():
    dig = make(seed=3, name='mock_excavation')
    dump = make(seed=3, name='mock_dump')
    assert [dig.start().duration_s for _ in range(5)] != [
        dump.start().duration_s for _ in range(5)
    ]


def test_failure_probability_does_not_shift_later_durations():
    plain = make(seed=5, probability=0.0)
    flaky = make(seed=5, probability=1.0)
    assert [plain.start().duration_s for _ in range(10)] == [
        flaky.start().duration_s for _ in range(10)
    ]
