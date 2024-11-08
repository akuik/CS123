import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from openai import OpenAI

client = OpenAI(api_key='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')
import numpy as np
import time
import io
import wave
import whisper as wh

# Set your OpenAI API key here

class CommandLinePublisher(Node):
    def __init__(self):
        super().__init__('command_line_publisher')

        # Create a publisher for the user query topic
        self.publisher_ = self.create_publisher(
            String,
            'user_query_topic',  # Replace with the topic name used in your GPT-4 node
            10
        )
        self.get_logger().info('Command Line Publisher Node has started.')
        self.model = wh.load_model("tiny")

    # TODO: Implement the publish_message method
    # message is a string that contains the user query. You can publish it using the publisher_ and its publish method
    def publish_message(self, message):
        # Create a String message and publish it
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        # Copy implementation from the command_line_publisher.py script

    def transcribe_audio_with_whisper(self, audio_data, sample_rate=16000):
        try:
            # Use the numpy array directly in Whisper's model for transcription
            print("Transcribing audio using Whisper model...")
            audio_float = audio_data.astype(np.float32) / 32768.0  # Convert int16 to float32
            response = self.model.transcribe(audio_float)
            self.get_logger().info(f"Transcription response: {response['text']}")
            return response['text']
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None


def record_audio(duration=5, sample_rate=16000):
    print("Recording audio...")
    audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
    for i in range(duration, 0, -1):
        print(f"Time left for recording: {i} seconds", end="\r", flush=True)
        time.sleep(1)
    sd.wait()  # Wait until the recording is finished
    print("Audio recording finished.")
    return np.squeeze(audio_data)

def audio_to_wav(audio_data, sample_rate=16000):
    """Convert numpy array audio to WAV format"""
    wav_io = io.BytesIO()
    with wave.open(wav_io, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_data.tobytes())
    wav_io.seek(0)
    return wav_io

def main(args=None):
    rclpy.init(args=args)

    # Create the command line publisher node
    command_publisher = CommandLinePublisher()

    # Set up the stream and audio processing
    print("Listening for speech every 8 seconds. Say 'exit' to stop.")

    try:
        # Continuously capture audio and process it
        while rclpy.ok():
            # Record 0.9 seconds of audio
            dur = 5 # TODO: Experiment with different durations
            audio_data = record_audio(duration=dur)

            # Transcribe audio using Whisper API
            user_input = command_publisher.transcribe_audio_with_whisper(audio_data)

            # If the user said 'exit', stop the loop
            if user_input and user_input.lower() == 'exit':
                print("Exiting the publisher.")
                break

            # Publish the recognized text
            if user_input:
                command_publisher.publish_message(user_input)

            # Allow ROS2 to process the message
            rclpy.spin_once(command_publisher, timeout_sec=0.1)

            # Delay for 0.9 seconds
            for i in range(20, 0, -1):
                print(f"Performing action; {i} seconds until the next recording starts", end="\r", flush=True)
                time.sleep(1)

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    # Clean up and shutdown
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
