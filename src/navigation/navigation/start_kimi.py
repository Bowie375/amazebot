import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

os.environ['OPENAI_API_KEY'] = "your key here"
from openai import OpenAI
client = OpenAI()


class Kimi(Node):
    """
        Kimi is a chatbot that can answer questions and give cmds.
        
        Kimi subscribes to the topic "/chat_input" and listens for incoming messages.
        When a message is received, Kimi sends it to the Moonshot AI API to generate a response.
        
        If the question is not about moving the robot, the response is then published 
        to the "/chat_output" topic; otherwise, the response is processed and published
        to the "/cmd_vel" topic.
    """

    def __init__(self):
        super().__init__("kimi")
        self.chat_input_subscription = self.create_subscription(
            String, "/chat_input", self.chat_input_callback, 10
        )
        self.chat_output_publisher = self.create_publisher(String, "/chat_output", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/model/amazebot/cmd_vel", 10)

        self.get_logger().info("Kimi is ready to chat!")

    def chat_input_callback(self, msg):
        question = msg.data.strip()
        response, cmd_type = generate_response(question)

        output_msg = String()

        if cmd_type == "S":
            # Publish response to /chat_output topic
            output_msg.data = response
            self.chat_output_publisher.publish(output_msg)
        elif cmd_type == "M":
            # Publish acknowledgement to /chat_output topic
            output_msg.data = f"好的，正在执行{response}"
            self.chat_output_publisher.publish(output_msg)

            cmdvel_msg = Twist()
            # Process response and publish to /cmd_vel topic
            if response == "小车前进":
                cmdvel_msg.linear.x = 0.5
            elif response == "小车后退":
                cmdvel_msg.linear.x = -0.5
            elif response == "小车左转":
                cmdvel_msg.angular.z = 0.5
            elif response == "小车右转":
                cmdvel_msg.angular.z = -0.5
            elif response == "小车停止":
                cmdvel_msg.linear.x = 0.0
                cmdvel_msg.angular.z = 0.0

            self.cmd_vel_publisher.publish(cmdvel_msg)
            print(f"Published cmd_vel message: {cmdvel_msg}")

def generate_response(question: str) -> str:
    completion = client.chat.completions.create(
        #model="moonshot-v1-8k",
		model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "you are a help AI assistant named amazebot, you can navigate using slam, "
                           "communicate with people and response to movement commands",
            },
            {
                "role": "user",
                "content": f"分析这个问题“{question}”，你不用输出分析过程，"
                            "只需要输出“其他指令”、“小车前进”、“小车后退”、“小车左转”、“小车右转”、“小车停止”之一；"
            },
        ],
    )

    result = completion.choices[0].message.content.strip()

    if result == "其他指令":
        completion = client.chat.completions.create(
            #model="moonshot-v1-8k",
	    	model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "you are a help AI assistant named amazebot, you can navigate using slam, "
                               "communicate with people and response to movement commands",
                },
                {
                    "role": "user",
                    "content": question,
                },
            ],)
        return completion.choices[0].message.content.strip(), "S"
            
    return result, "M"

def main(args=None):
    rclpy.init(args=args)
    kimi = Kimi()
    rclpy.spin(kimi)
    kimi.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()