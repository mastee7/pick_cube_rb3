#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

import ast
import time
import requests
import json



class Rb3Publisher(Node):
    def __init__(self):
        super().__init__('rb3_message_publisher')
        self.coord_sub = self.create_subscription(
            Float64MultiArray,
            'coords_topic',
            self.timer_callback,
            10 # QoS profile depth
            # Consider using specific QoS profiles if needed, e.g. rclpy.qos.qos_profile_sensor_data
        )

        command = ParameterDescriptor(description='Command')
        self.declare_parameter('command', 'Move Red Cube to Starting Point', command)
        self.command = self.get_parameter('command').value
 
        self.publisher_ = self.create_publisher(Float64MultiArray, 'rb3_topic', 10) # Topic name is 'rb3_topic'
        timer_period = 1.0  # seconds (publish every 1 second)
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.get_logger().info('RB3 Publisher Node started. Publishing to "rb3_topic".')
        self.RUNNING = True

    def timer_callback(self, sub_msg):
        data = sub_msg.data
        # Things to ask to professor
        if self.RUNNING: 
            valid, coordinate_data = self.run_language_model(data)
            print(f"@@@@@@@@@@@@@@@@@@@@ {coordinate_data}, {type(coordinate_data)}")
            if valid : 
                msg = Float64MultiArray()
                msg.data = coordinate_data  #f'Hello from RB3! Count: {self.counter} @ {time.time()}'
                self.RUNNING = False
                self.publisher_.publish(msg)

                self.get_logger().info(f'Publishing: "{msg.data}"')
            elif valid == False:
                self.get_logger().info(f'invalid data {coordinate_data} for {self.command}')
            self.counter += 1

    def run_language_model(self, msg, cmd=""):
        url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"
        api_key = "YOUR_API_KEY"  # Replace with your actual API key
        headers = {'Content-Type': 'application/json'}

        instruct_template = """
        <instruct> {instruct_text} </instruct>
        """

        information_template = """
        <information>
        {information_text}
        </information>
        """

        command_template = """
        <command>
        {command_text}
        </command>
        """

# --- Define your variable content here ---
        my_instruct = "Based on the provided information and the subsequent command, identify the coordinates for two distinct locations: the object to be manipulated and its target destination. Return these coordinates in a JSON format with the keys cube_loc and desti_loc. if you don't have sufficient cube information please return value \'Nan\'"

        msg = self.transform(msg)
        my_information = """
        Map boundaries: Start (413, 245), End (924, 590)
        Detected cube locations: 
        """ + msg #Green: ((482+531)/2, (489+541)/2) Yellow: ((751+811)/2, (278+323)/2)

        my_command = self.command #cmd#"Pick up the Yellow cube and place it at the start."
# --- End of variable definitions ---

        prompt = instruct_template.format(instruct_text=my_instruct) + "\n\n" + \
                 information_template.format(information_text=my_information) + "\n\n" + \
                 command_template.format(command_text=my_command)

        print(prompt)
        payload = {
            "contents": [
                {
                    "parts": [{"text": prompt}]
                }
            ]
        }

        params = {'key': api_key}

        try:
            response = requests.post(url, headers=headers, params=params, data=json.dumps(payload))
            response.raise_for_status()
            print(response.json())
        except requests.exceptions.RequestException as e:
            print(f"An error occurred: {e}")

        l_msg = response.json()['candidates'][0]['content']['parts'][0]['text']
        msg_match = l_msg.replace('```json', '').replace('```', '').strip()
        if 'Nan' in msg_match: return False, []


        output = json.loads(msg_match)
        target, destination = ast.literal_eval(output['cube_loc']), ast.literal_eval(output['desti_loc'])
        print(target, destination)
        target, destination = list(target), list(destination)
        print(target, destination)
        msg_match = [float(target[0]), float(target[1]), float(destination[0]), float(destination[1])]
        print(msg_match)
        print('-------')
        return True, msg_match #output #target, destination

    def transform(self, msg):
          classes = {
                    0: 'Yellow Cube',
                    1: 'Red Cube',
                    2: 'Blue Cube',
                    3: 'Green Cube',
                  }
          sublist_size = 3
          data = ''
          for i in range(0, len(msg), sublist_size):
              data += f" {classes[msg[i]]}: ({msg[i+1]}, {msg[i+2]})\n" 
          return data
def main(args=None):
    rclpy.init(args=args)
    rb3_publisher = Rb3Publisher()
    try:
        rclpy.spin(rb3_publisher)
    except KeyboardInterrupt:
         pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        rb3_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
