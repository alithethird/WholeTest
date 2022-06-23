"""This python file is for Control Hub to achieve communication between Test Robot and UI"""
from threading import Event
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robot_interface.action import ROBOT as ROBOT_Action
from std_msgs.msg import String
from example_interfaces.srv import Trigger

class ClientNode(Node):
    """ This is the main ROS2 communication class"""
    def __init__(self):
        super().__init__("clientell")
        self.service_done_event = Event()
        server_cb_group = ReentrantCallbackGroup()
        action_cb_group = None
        self._action_client = ActionClient(self, ROBOT_Action,
                                           'test_robot_server',
                                           callback_group=action_cb_group)
        self.subscriber_ = self.create_subscription(String,
                                                    "ui_control_hub_topic",
                                                    self.callback_ui_subscription,
                                                    10)
        self.server_ = self.create_service(Trigger,
                                           "control_hub_button_server",
                                           self.callback_button_state,
                                           callback_group=server_cb_group)
        self.get_logger().info("control_hub_button_server server has been started.")
        self.get_logger().info("control hub is listening to ui_control_hub_topic")
        self.send_goal("20,low")
        self.result = "empty"
        self._get_result_future = 0

    def callback_button_state(self, request, response):
        """ This callback function runs when a button read request comes from UI 
        :param request: request coming from UI to read button state"""
        if not self._action_client.wait_for_server(timeout_sec=1):
            self.get_logger().error('No action server availible!!!')
            return response

        self.service_done_event.clear()

        event = Event()

        def done_callback(future):
            """ This callback function runs when
            the button state response is acquired from Test Robot """
            if str(future.result().result.value) == '1':
                response.message = "pressed"
            else:
                response.message = "not pressed"

            response.success = True
            nonlocal event
            event.set()

        def goal_callback(future):
            """ This callback function runs when
            goal state is acquired from Test Robot """
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(done_callback)


        request = GPIO_Action.Goal() # request is initialized
        request.gpio = str("26,read") # request is set to read the pin 26
        future = self._action_client.send_goal_async(request) # sending the request
        future.add_done_callback(goal_callback) # adding a callback function to process response

        event.wait() # wait for event to set free
        self.get_logger().info('Result: {0}'.format(response)) # print the result to terminal for debugging
        return response # return response to UI



    def callback_ui_subscription(self, msg):
        """ This callback function is run when
        a message comes from UI through topic.
        When a message is received that message is
        send to the Test Robot to activate or deactivate
        the LED. """
        self.get_logger().info(msg.data)
        self.send_goal(str(msg.data))

    def send_goal(self, gpio):
        """ This function is used to send the gpio data
        to the Test Robot """
        goal_msg = GPIO_Action.Goal()
        goal_msg.gpio = str(gpio)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        """ This callback function runs when
        the button state response is acquired from Test Robot """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ This callback function runs when
        the LED state response is acquired from Test Robot """
        self.result = future.result().result
        self.get_logger().info('Result: {0}'.format(self.result.value))
        self.service_done_event.set()


def main(args=None):
    """ This is the main function that starts the ClientNode """
    rclpy.init(args=args)

    action_client = ClientNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor)


if __name__ == '__main__':
    main()
