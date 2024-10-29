from threading import Lock

import rclpy
import rclpy.node
from rclpy.qos import QoSPresetProfiles

from compas_rrc_ros_interfaces import msg


class SequenceCheckModes(object):
    NONE = 'none'
    ALL = 'all'
    INCOMING = 'incoming'
    OUTGOING = 'outgoing'


class RobotMessageTopicProvider(object):
    def __init__(self, topic_name_sub, topic_name_pub, streaming_interface, robot_state, node: rclpy.node.Node, options=None):
        super(RobotMessageTopicProvider, self).__init__()

        self._publish_lock = Lock()
        self._receive_lock = Lock()
        self._last_published_id = 0
        self._last_received_id = 0
        self.sequence_check_mode = SequenceCheckModes.NONE
        self.node = node
        self.logger = node.get_logger()

        if options:
            self.sequence_check_mode = options.get('sequence_check_mode', SequenceCheckModes.NONE)

            if self.sequence_check_mode not in (SequenceCheckModes.NONE, SequenceCheckModes.ALL, SequenceCheckModes.INCOMING, SequenceCheckModes.OUTGOING):
                raise Exception('Unsupported sequence check mode. Value={}'.format(self.sequence_check_mode))

        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

        # TODO: Verify topic queue sizes
        self.subscriber = self.node.create_subscription(msg.RobotMessage, topic_name_sub, self.ros_to_robot_handler, QoSPresetProfiles.get_from_short_key("default"))
        self.publisher = self.node.create_publisher(msg.RobotMessage, topic_name_pub, 1)

        self.robot_state.on_message(self.robot_to_ros_handler)
        self.robot_state.on_socket_broken(self._reset_sequence_id)

        self.logger.info(f'Topic provider started. Subscribed to {topic_name_sub}, publishing to {topic_name_pub}')

    def _reset_sequence_id(self):
        with self._publish_lock:
            self._last_published_id = 0

        with self._receive_lock:
            self._last_received_id = 0

    def ros_to_robot_handler(self, ros_message):
        """Handle messages from ROS topics to the robot controller"""
        try:
            with self._publish_lock:
                if self.sequence_check_mode in (SequenceCheckModes.OUTGOING, SequenceCheckModes.ALL):
                    if ros_message.sequence_id != self._last_published_id + 1:
                        raise Exception('Received out of order (ROS -> Controller). Received={}, Last sequence id={}'.format(ros_message.sequence_id, self._last_published_id))
                self.streaming_interface.execute_instruction(ros_message)
        except Exception as e:
            self.logger.error(e)
            raise e
        finally:
            self._last_published_id = ros_message.sequence_id

    def robot_to_ros_handler(self, ros_message):
        """Handle messages from the robot controller to ROS topic."""
        with self._receive_lock:
            try:
                if self.sequence_check_mode in (SequenceCheckModes.INCOMING, SequenceCheckModes.ALL):
                    if ros_message.sequence_id != self._last_received_id + 1:
                        raise Exception('Received out of order (Controller -> ROS). Received={}, Last sequence id={}'.format(ros_message.sequence_id, self._last_received_id))
                self.publisher.publish(ros_message)
            except Exception as e:
                self.logger.error(e)
                raise e
            finally:
                self._last_received_id = ros_message.sequence_id

    def disconnect(self):
        try:
            self.node.destroy_subscription(self.subscriber)
            self.node.destroy_publisher(self.publisher)
            self.logger.info("Topic provider disconnected")
        except Exception as e:
            self.logger.error(e)
