import os
import sys

import threading
import subprocess
from playsound import playsound

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Microphone(Node):
    """
        this node is responsible for listening and speaking

        listening:
            It waits for a keyboard input, when 'r' is pressed, call iat api
            to record the audio, when 'q' is pressed, stop recording. If 'a'
            is pressed, it will listen to a given audio file.
            In both cases, publish the audio data to '/chat_input' topic.

        speaking:
            Listen to '/chat_output' topic, when a message is received,
            call tts api to synthesize the audio and play it.
    """

    def __init__(self):
        super().__init__('microphone')
        self.chat_input_publisher = self.create_publisher(
            String, '/chat_input', 10)
        self.chat_output_subscription = self.create_subscription(
            String, '/chat_output', self.speak, 10)
        
        base_dir = os.environ.get('AMAZEBOT_BASE_DIR')
        self.iat_api = os.path.join(base_dir, 'src/xunfei/bin/iat_online_record_sample')
        self.tts_api = os.path.join(base_dir, 'src/xunfei/bin/tts_online_sample')
        
        # set a log path for recording
        self.log_path = os.path.join(base_dir, 'log/microphone')
        os.makedirs(self.log_path, exist_ok=True)

        self.listen_thread = threading.Thread(target=self.listen)
        self.listen_thread.start()
        self.get_logger().info('Microphone node started')

    def listen(self):
        is_recording = False
        output_path = os.path.join(self.log_path, 'chat_input.txt')
        print('Press r to start recording, q to quit, a to listen to audio file:')
        while True:
            key = input()
            if key == 'r':
                if is_recording:
                    self.get_logger().info('Recording already started')
                    continue
                is_recording = True
                self.get_logger().info('Recording started')
                process = subprocess.Popen([self.iat_api, output_path], 
                                           text=True)
            elif key == 'q':
                if not is_recording:
                    self.get_logger().info('Recording not started')
                    continue
                _ , _ = process.communicate('q')
                process.wait()
                
                output_msg = String()
                with open(output_path, 'r') as f:
                    output_msg.data = f.read()
                self.chat_input_publisher.publish(output_msg)
                
                is_recording = False
                self.get_logger().info('Recording stopped')
            elif key == 'a':
                if is_recording:
                    self.get_logger().info('Recording already started')
                    continue
                audio_file = input('Enter audio file path: ')
                if not os.path.exists(audio_file):
                    self.get_logger().info('Audio file not found')
                    continue
                result = subprocess.run([self.iat_api, "0", audio_file], 
                                        capture_output=True, text=True)
                self.get_logger().info("Audio file played")
                output_msg = String()
                output_msg.data = result.stdout
                self.get_logger().info("output_msg: {}".format(output_msg.data))
                self.chat_input_publisher.publish(output_msg)

    def speak(self, msg):
        output_path = os.path.join(self.log_path, 'output.wav')
        subprocess.run([self.tts_api, output_path, msg.data])
        playsound(output_path)

def main(args=None):
    rclpy.init(args=args)
    microphone = Microphone()
    rclpy.spin(microphone)
    microphone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()