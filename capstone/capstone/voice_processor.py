#!/usr/bin/env python3

"""
Voice Processor Node for Capstone Project

This node processes audio input to convert speech to text using Whisper.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image

import torch
import whisper
import numpy as np
from scipy.io import wavfile


class VoiceProcessorNode(Node):
    """
    A node that processes audio input and converts speech to text using Whisper.
    """

    def __init__(self):
        super().__init__('voice_processor')

        # Initialize Whisper model
        self.get_logger().info('Loading Whisper model...')
        try:
            self.model = whisper.load_model("base")
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=10)

        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            qos_profile
        )

        self.text_pub = self.create_publisher(
            String,
            'transcribed_text',
            qos_profile
        )

        self.command_pub = self.create_publisher(
            String,
            'spoken_command',
            qos_profile
        )

        # Audio processing parameters
        self.sample_rate = 16000  # Expected sample rate for Whisper
        self.audio_buffer = np.array([], dtype=np.int16)

        self.get_logger().info('Voice Processor node initialized')

    def audio_callback(self, msg):
        """
        Callback for audio data
        """
        if self.model is None:
            return

        try:
            # Convert audio data to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16)

            # Append to buffer (in a real system, you might want to process chunks)
            self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

            # Process when buffer is large enough (0.5 seconds of audio)
            if len(self.audio_buffer) >= self.sample_rate // 2:
                # Transcribe the audio
                text = self.transcribe_audio(self.audio_buffer)

                if text and text.strip():
                    # Publish transcribed text
                    text_msg = String()
                    text_msg.data = text.strip()
                    self.text_pub.publish(text_msg)

                    # Also publish as command
                    cmd_msg = String()
                    cmd_msg.data = text.strip()
                    self.command_pub.publish(cmd_msg)

                    self.get_logger().info(f'Transcribed: {text}')

                # Reset buffer
                self.audio_buffer = np.array([], dtype=np.int16)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def transcribe_audio(self, audio_data):
        """
        Transcribe audio data using Whisper
        """
        try:
            # Normalize audio data to float32 in range [-1, 1]
            audio_float = audio_data.astype(np.float32) / 32768.0

            # Transcribe using Whisper
            result = self.model.transcribe(audio_float, fp16=False)
            return result["text"]

        except Exception as e:
            self.get_logger().error(f'Error in transcription: {e}')
            return ""


def main(args=None):
    rclpy.init(args=args)

    voice_processor = VoiceProcessorNode()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()