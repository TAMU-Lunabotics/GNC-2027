from luna_interfaces.action import Dump
from luna_sim.actuators.timed_action_node import TimedActionNode, spin


class MockDump(TimedActionNode):
    def __init__(self):
        super().__init__(
            'mock_dump', Dump, '/dump', 'jam',
            {'duration_s': 10.0, 'jitter_s': 2.0, 'feedback_hz': 2.0,
             'failure_probability': 0.0, 'seed': 0})


def main(args=None):
    spin(MockDump, args)
