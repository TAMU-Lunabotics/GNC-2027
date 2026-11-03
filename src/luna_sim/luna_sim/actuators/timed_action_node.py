import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from luna_sim.actuators.timed_action import CANCELED, SUCCEEDED, TimedAction, make_rng


class TimedActionNode(Node):
    def __init__(self, node_name, action_type, action_name, failure_reason, defaults):
        super().__init__(node_name)

        for key, value in defaults.items():
            self.declare_parameter(key, value)
        get = lambda key: self.get_parameter(key).value

        self.feedback_hz = float(get('feedback_hz'))
        seed = int(get('seed'))
        self.logic = TimedAction(
            duration_s=float(get('duration_s')),
            jitter_s=float(get('jitter_s')),
            failure_probability=float(get('failure_probability')),
            failure_reason=failure_reason,
            rng=make_rng(seed, node_name),
        )

        self.action_type = action_type
        self.busy = False
        self.server = ActionServer(
            self, action_type, action_name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f'{node_name} ready on {action_name}')

    def on_goal(self, goal_request):
        if self.busy:
            self.get_logger().warn('goal rejected: one is already running')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        result = self.action_type.Result()
        feedback = self.action_type.Feedback()
        rate = self.create_rate(self.feedback_hz, self.get_clock())
        try:
            plan = self.logic.start(float(goal_handle.request.max_duration_s))
            self.get_logger().info(f'started: duration {plan.duration_s:.1f} s')
            start = self.get_clock().now()

            while True:
                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                verdict = plan.check(elapsed, goal_handle.is_cancel_requested)
                if verdict is not None:
                    break
                feedback.elapsed_s = float(elapsed)
                goal_handle.publish_feedback(feedback)
                rate.sleep()

            result.success = verdict == SUCCEEDED
            if verdict == SUCCEEDED:
                goal_handle.succeed()
            elif verdict == CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().info(f'finished: {verdict} after {elapsed:.1f} s')
            return result
        finally:
            self.destroy_rate(rate)
            self.busy = False


def spin(node_class, args=None):
    rclpy.init(args=args)
    node = node_class()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
