"""Timing logic shared by mock_excavation and mock_dump. No ROS imports."""
import random
from dataclasses import dataclass

SUCCEEDED = 'succeeded'
CANCELED = 'canceled'
TIMEOUT = 'timeout'


def make_rng(seed, name):
    return random.Random(f'{seed}:{name}')


@dataclass(frozen=True)
class Plan:
    duration_s: float
    end_s: float
    outcome: str

    def check(self, elapsed_s, cancel_requested=False):
        if cancel_requested:
            return CANCELED
        if elapsed_s >= self.end_s:
            return self.outcome
        return None


class TimedAction:
    def __init__(self, duration_s, jitter_s, failure_probability, failure_reason, rng):
        self.duration_s = duration_s
        self.jitter_s = jitter_s
        self.failure_probability = failure_probability
        self.failure_reason = failure_reason
        self.rng = rng

    def start(self, max_duration_s=0.0):
        # always three draws so changing failure_probability doesn't shift later goals
        jitter = self.rng.uniform(0.0, self.jitter_s)
        roll = self.rng.random()
        fraction = self.rng.uniform(0.2, 0.8)

        duration = self.duration_s + jitter
        end_s, outcome = duration, SUCCEEDED

        if roll < self.failure_probability:
            end_s, outcome = fraction * duration, self.failure_reason

        if max_duration_s > 0.0 and end_s > max_duration_s:
            end_s, outcome = max_duration_s, TIMEOUT

        return Plan(duration, end_s, outcome)
