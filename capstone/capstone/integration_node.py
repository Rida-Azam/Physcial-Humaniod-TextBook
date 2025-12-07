#!/usr/bin/env python3

"""
Integration Node for Capstone Project

This node integrates voice processing (Whisper), LLM planning, and ROS execution
into a cohesive system for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from capstone_interfaces.msg import CommandStructure, TaskPlan, ExecutionStatus

import threading
import time
from enum import Enum


class SystemState(Enum):
    """Overall system state"""
    IDLE = 0
    LISTENING = 1
    PROCESSING = 2
    PLANNING = 3
    EXECUTING = 4
    COMPLETED = 5
    ERROR = 6


class IntegrationNode(Node):
    """
    Main integration node that coordinates all components.
    """

    def __init__(self):
        super().__init__('integration_node')

        # Initialize system state
        self.system_state = SystemState.IDLE
        self.last_command = ""
        self.current_plan = None
        self.execution_status = None

        # Callback groups for concurrent processing
        self.voice_cb_group = MutuallyExclusiveCallbackGroup()
        self.nlu_cb_group = MutuallyExclusiveCallbackGroup()
        self.planning_cb_group = MutuallyExclusiveCallbackGroup()
        self.execution_cb_group = MutuallyExclusiveCallbackGroup()

        # Publishers
        qos_profile = QoSProfile(depth=10)

        self.status_pub = self.create_publisher(
            String,
            'system_status',
            qos_profile
        )

        self.command_pub = self.create_publisher(
            String,
            'spoken_command',
            qos_profile
        )

        # Subscribers - these would connect to the individual component nodes
        self.transcription_sub = self.create_subscription(
            String,
            'transcribed_text',
            self.transcription_callback,
            qos_profile,
            callback_group=self.voice_cb_group
        )

        self.structure_sub = self.create_subscription(
            CommandStructure,
            'structured_command',
            self.structure_callback,
            qos_profile,
            callback_group=self.nlu_cb_group
        )

        self.plan_sub = self.create_subscription(
            TaskPlan,
            'task_plan',
            self.plan_callback,
            qos_profile,
            callback_group=self.planning_cb_group
        )

        self.execution_status_sub = self.create_subscription(
            ExecutionStatus,
            'execution_status',
            self.execution_status_callback,
            qos_profile,
            callback_group=self.execution_cb_group
        )

        # Timer for system state updates
        self.status_timer = self.create_timer(1.0, self.publish_system_status)

        # Initialize components
        self.initialize_components()

        self.get_logger().info('Integration node initialized and ready')

    def initialize_components(self):
        """
        Initialize all system components
        """
        self.get_logger().info('Initializing system components...')

        # Here we would typically check if all required nodes are running
        # For demonstration, we'll just log the initialization
        self.get_logger().info('System components initialized')

    def transcription_callback(self, msg):
        """
        Callback for transcribed text
        """
        self.get_logger().info(f'Received transcription: {msg.data}')

        self.system_state = SystemState.PROCESSING
        self.last_command = msg.data

        # Publish command for other components
        self.command_pub.publish(msg)

        self.get_logger().info('Command forwarded for NLU processing')

    def structure_callback(self, msg):
        """
        Callback for structured commands
        """
        self.get_logger().info(f'Received structured command: {msg.intent}')

        self.system_state = SystemState.PLANNING

        self.get_logger().info('Command structure received, waiting for plan...')

    def plan_callback(self, msg):
        """
        Callback for task plans
        """
        self.get_logger().info(f'Received task plan with {len(msg.steps)} steps')

        self.current_plan = msg
        self.system_state = SystemState.EXECUTING

        self.get_logger().info('Task plan received, starting execution...')

    def execution_status_callback(self, msg):
        """
        Callback for execution status
        """
        self.execution_status = msg

        if msg.state == 4:  # Assuming 4 is COMPLETED based on our enum
            self.system_state = SystemState.COMPLETED
            self.get_logger().info('Task execution completed successfully')
        elif msg.state == 3:  # Assuming 3 is ERROR
            self.system_state = SystemState.ERROR
            self.get_logger().error(f'Task execution failed: {msg.message}')
        else:
            self.system_state = SystemState.EXECUTING
            self.get_logger().info(f'Execution in progress: {msg.message}')

    def publish_system_status(self):
        """
        Publish overall system status
        """
        status_msg = String()
        status_msg.data = f"STATE: {self.system_state.name} | LAST_CMD: {self.last_command[:50] if self.last_command else 'None'}"
        self.status_pub.publish(status_msg)

        self.get_logger().debug(f'System status: {self.system_state.name}')

    def reset_system(self):
        """
        Reset the system to initial state
        """
        self.system_state = SystemState.IDLE
        self.last_command = ""
        self.current_plan = None
        self.execution_status = None

        self.get_logger().info('System reset to IDLE state')

    def start_listening(self):
        """
        Start listening for voice commands
        """
        self.system_state = SystemState.LISTENING
        self.get_logger().info('System listening for voice commands...')

    def is_system_idle(self):
        """
        Check if system is idle and ready for new commands
        """
        return self.system_state == SystemState.IDLE or self.system_state == SystemState.COMPLETED


def main(args=None):
    rclpy.init(args=args)

    integration_node = IntegrationNode()

    # Start in listening mode
    integration_node.start_listening()

    try:
        rclpy.spin(integration_node)
    except KeyboardInterrupt:
        integration_node.get_logger().info('Interrupted by user')
    finally:
        integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()