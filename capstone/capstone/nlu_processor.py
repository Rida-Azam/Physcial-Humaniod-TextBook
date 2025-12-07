#!/usr/bin/env python3

"""
Natural Language Understanding (NLU) Processor Node for Capstone Project

This node processes natural language commands and extracts structured meaning.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from capstone_interfaces.msg import CommandStructure  # Assuming a custom message type

import spacy
from transformers import pipeline
import json
import re


class NLUProcessorNode(Node):
    """
    A node that processes natural language commands and extracts structured meaning.
    """

    def __init__(self):
        super().__init__('nlu_processor')

        # Initialize NLP model
        self.get_logger().info('Loading NLP model...')
        try:
            # Using a simple rule-based approach for demonstration
            # In practice, you'd use a pre-trained model
            self.nlp = spacy.load("en_core_web_sm")
            self.get_logger().info('NLP model loaded successfully')
        except OSError:
            self.get_logger().warn('spaCy model not found, using simple pattern matching')
            self.nlp = None

        # Initialize transformers pipeline for more advanced NLU
        try:
            self.fill_mask = pipeline("fill-mask", model="bert-base-uncased")
            self.get_logger().info('Transformer pipeline loaded')
        except Exception as e:
            self.get_logger().warn(f'Could not load transformer pipeline: {e}')
            self.fill_mask = None

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=10)

        self.command_sub = self.create_subscription(
            String,
            'spoken_command',
            self.command_callback,
            qos_profile
        )

        self.structure_pub = self.create_publisher(
            CommandStructure,
            'structured_command',
            qos_profile
        )

        self.interpretation_pub = self.create_publisher(
            String,
            'command_interpretation',
            qos_profile
        )

        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'navigate to (.+)',
                r'walk to (.+)',
                r'travel to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'take (.+)',
                r'grab (.+)',
                r'lift (.+)',
                r'hold (.+)'
            ],
            'action_object': [
                r'move (.+) to (.+)',
                r'put (.+) on (.+)',
                r'place (.+) at (.+)'
            ]
        }

        # Define object and location keywords
        self.objects = [
            'cup', 'box', 'ball', 'bottle', 'book', 'plate', 'bowl', 'chair', 'table',
            'red cup', 'blue box', 'green ball', 'yellow bottle', 'white book'
        ]

        self.locations = [
            'kitchen', 'living room', 'bedroom', 'office', 'dining room', 'bathroom',
            'table', 'chair', 'couch', 'shelf', 'counter', 'desk'
        ]

        self.get_logger().info('NLU Processor node initialized')

    def command_callback(self, msg):
        """
        Callback for natural language commands
        """
        command = msg.data.lower().strip()

        self.get_logger().info(f'Processing command: {command}')

        # Parse the command
        parsed_command = self.parse_command(command)

        # Create and publish structured command
        structure_msg = self.create_structure_message(parsed_command)
        self.structure_pub.publish(structure_msg)

        # Publish interpretation as string
        interpretation_msg = String()
        interpretation_msg.data = json.dumps(parsed_command, indent=2)
        self.interpretation_pub.publish(interpretation_msg)

        self.get_logger().info(f'Parsed command: {parsed_command}')

    def parse_command(self, command):
        """
        Parse a natural language command into structured components
        """
        # Initialize result structure
        result = {
            'raw_command': command,
            'intent': 'unknown',
            'entities': {
                'objects': [],
                'locations': [],
                'actions': []
            },
            'confidence': 0.0
        }

        # Extract intent and entities using pattern matching
        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command)
                if match:
                    result['intent'] = intent
                    result['confidence'] = 0.9  # High confidence for pattern match

                    # Extract captured groups
                    groups = match.groups()
                    for group in groups:
                        # Try to classify the group as object or location
                        if any(obj in group for obj in self.objects):
                            result['entities']['objects'].append(group)
                        elif any(loc in group for loc in self.locations):
                            result['entities']['locations'].append(group)
                        else:
                            # If not classified, add to both as potential
                            result['entities']['objects'].append(group)
                            result['entities']['locations'].append(group)

                    break  # Break after first match for this intent

            if result['intent'] != 'unknown':
                break

        # If no pattern matched, try simple keyword extraction
        if result['intent'] == 'unknown':
            result['confidence'] = 0.5  # Lower confidence for keyword extraction

            # Extract objects
            for obj in self.objects:
                if obj in command:
                    result['entities']['objects'].append(obj)

            # Extract locations
            for loc in self.locations:
                if loc in command:
                    result['entities']['locations'].append(loc)

            # Extract actions
            if any(action in command for action in ['go', 'move', 'navigate', 'walk', 'travel']):
                result['entities']['actions'].append('navigate')
            if any(action in command for action in ['pick', 'grasp', 'take', 'grab', 'lift', 'hold']):
                result['entities']['actions'].append('manipulate')

        return result

    def create_structure_message(self, parsed_command):
        """
        Create a structured message from the parsed command
        """
        structure_msg = CommandStructure()
        structure_msg.header.stamp = self.get_clock().now().to_msg()
        structure_msg.header.frame_id = 'base_link'

        # Set intent
        structure_msg.intent = parsed_command['intent']

        # Set entities
        structure_msg.entities.objects = parsed_command['entities']['objects']
        structure_msg.entities.locations = parsed_command['entities']['locations']
        structure_msg.entities.actions = parsed_command['entities']['actions']

        # Set confidence
        structure_msg.confidence = parsed_command['confidence']

        return structure_msg


def main(args=None):
    rclpy.init(args=args)

    nlu_processor = NLUProcessorNode()

    try:
        rclpy.spin(nlu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        nlu_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()