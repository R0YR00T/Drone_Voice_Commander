##############################################
##                                          ##
##                                          ##
##      Name: Rany Daioub                   ##
##      ID: 219547228                       ##
##      Project: Drone Voice Commander      ##
##      Code: services_client.py            ##
##                                          ##
##                                          ##
##############################################



############### IMPORTING LIBRARIES ################

import sys
import pyaudio
import rclpy
import time


from drone_interfaces.srv import Move
from std_srvs.srv import Empty
from vosk import Model, KaldiRecognizer
from rclpy.node import Node

####################################################






################### CLIENT CLASS ###################

class Client(Node):
    
    def __init__(self):
        super().__init__('client_async')
        self.model = Model(r"/home/rany/Downloads/vosk-model-en-us-0.22")
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.mic = pyaudio.PyAudio()
        self.listening = False


    
    def get_command(self):
        self.listening = True
        stream = self.mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)

        while self.listening:
            stream.start_stream()
            try:
                input = stream.read(4096)
                if self.recognizer.AcceptWaveform(input):
                    speach = self.recognizer.Result()
                    result = speach[14:-3]
                    self.listening = False
                    stream.close()
                    return result
            except OSError:
                pass

####################################################






####################################################
def main(args=None):
    rclpy.init(args=args)
    client = Client()
    node = rclpy.create_node('drone_service_client')

    while True:
        
        print("Waiting ...")
        voice_response = client.get_command()
        

        #################################
        if voice_response == "go":
            cli = node.create_client(Empty, 'takeoff')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("takeoff successful")

            time.sleep(4)


        #######################################
        if voice_response == "land":
            print("Landing")
            cli = node.create_client(Empty, 'land')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("land successful")


        #######################################
        if voice_response == "front":
            print("Moving forward")
            cli = node.create_client(Move, 'move_forward')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("move forward successful")

            time.sleep(4)


        #######################################
        if voice_response == "back":
            print("Moving backward")
            cli = node.create_client(Move, 'move_backward')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("move backward successful")

            time.sleep(4)


        #######################################
        if voice_response == "left":
            print("Moving left")
            cli = node.create_client(Move, 'left')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("move left successful")

            time.sleep(4)
        

        #######################################
        if voice_response == "right":
            print("Moving right")
            cli = node.create_client(Move, 'right')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("move right successful")

            time.sleep(4)


        #######################################
        if voice_response == "flip":
            print("Flipping backwards")
            cli = node.create_client(Empty, 'flip_backward')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)

            try:
                result = future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info("FLipping backward successful")

            time.sleep(4)


        #################################
        else:

            print("I did not understand")
            print(voice_response)



    # client.destroy_node()
    # rclpy.shutdown()



#####################################################
if __name__ == '__main__':
    main()







