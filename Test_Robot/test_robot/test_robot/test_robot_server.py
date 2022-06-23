""" This is an example code that is included in the pi_gpio module for ROS2
This example code is modified to work well with Control Hub Test Code """
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interface.action import ROBOT as ROBOT_Action
import RPi.GPIO as GPIO

class RaspberryPIGPIO():
    """ This class is used for controlling Raspberry Pi GPIO """
    def __init__(self, pin_id, pin_type):
        self.pin_id = pin_id
        self.pin_type = pin_type.rstrip()
        GPIO.setwarnings(False)
        #Use Broadcom pin-numbering scheme
        GPIO.setmode(GPIO.BCM)
        if self.pin_type == "in":
            GPIO.setup(pin_id, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #Set pin as input
            print("Setting GPIO " + str(self.pin_id) + "-" + self.pin_type)

        elif self.pin_type == "out":
            GPIO.setup(pin_id, GPIO.OUT) #Set pin as output
            print("Setting GPIO " + str(self.pin_id) + "-" + self.pin_type)

        time.sleep(0.1)

    def set_pin(self, value):
        """ This function is used to set pins high or low """
        if value == 1:
            GPIO.output(self.pin_id, GPIO.HIGH) #Set pin High-1
        elif value == 0:
            GPIO.output(self.pin_id, GPIO.LOW) #Set pin Low-0


class ROBOTActionServer(Node):
    """ This class is used to handle ROS2 Action server for
    communication with Control Hub """
    def __init__(self):
        super().__init__('test_robot_server')

        self.pin_dic = {}

        self.pin_dic["20"] = RaspberryPIGPIO(20, 'out')
        self.pin_dic["26"] = RaspberryPIGPIO(26, 'in')

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        #Node, action_type, action_name, execute_callback
        self._action_server = ActionServer(
            self,
            ROBOT_Action,
            'test_robot_server',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        """ This function cleans the node """
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """ This callback function is used to create
        goal responce accept message """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Executes the goal."""
        self.get_logger().info('Executing goal...')

        # Populate goal message
        goal_msg = goal_handle.request.gpio

        # Populate feedback message
        feedback_msg = ROBOT_Action.Feedback()
        feedback_msg.feedback = 1

        # Populate result message
        result = ROBOT_Action.Result()

        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            return ROBOT_Action.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return ROBOT_Action.Result()

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)

        # get the pin ide and action pin_type
        pin_id, action_type = goal_msg.split(',')

        if action_type == "high":
            self.pin_dic[pin_id].set_pin(1)
            time.sleep(0.1)
            result.value = 3

        elif action_type == "low":
            self.pin_dic[pin_id].set_pin(0)
            time.sleep(0.1)
            result.value = 3

        elif action_type == "read":
            result.value = GPIO.input(int(pin_id))

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    action_server = ROBOTActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()

    GPIO.cleanup()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
