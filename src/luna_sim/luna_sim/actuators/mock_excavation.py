from luna_interfaces.action import Dig
from luna_sim.actuators.timed_action_node import TimedActionNode, spin


class MockExcavation(TimedActionNode):
    def __init__(self):
        super().__init__(
            'mock_excavation', Dig, '/dig', 'stall',
            {'duration_s': 20.0, 'jitter_s': 3.0, 'feedback_hz': 2.0,
             'failure_probability': 0.0, 'seed': 0})


def main(args=None):
    spin(MockExcavation, args)
