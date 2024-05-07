#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import Llama
from audio_common_msgs.msg import AudioData
import subprocess
from threading import Thread
import time
import select
import base64
import numpy as np

conda_env = "transformer-tts"
#kiirkirj_command = "arecord -r 16000 -t raw -f S16_LE -c 1 - | play -t raw -e signed-integer -b 16 -c 1 -r 16k -"
kiirkirj_command = "docker exec -i kiirkirjutaja python main.py -"

class semubotSpeechNode(Node):
    def __init__(self):
        super().__init__('semubotSpeechNode')
        self.kiirkirjutaja_process = None
        self.output_detected = True
        
        # create client to send service request to Llama node
        self.LLama_client = self.create_client(Llama, 'Llama_service')

        # TODO create subscriber for microphone audio stream topic
        self.debug_publisher = self.create_publisher(String, 'Speech_topic', 10)
        # Start Speech synthesis
        self.start_speech_recogintion(kiirkirj_command)
        
        self.counter = 0

    def start_speech_recogintion(self, program_cmd):
        # Start kiirkirjutaja in container
        self.kiirkirjutaja_process = subprocess.Popen(program_cmd, shell=True, stdout=subprocess.PIPE, stdin = subprocess.PIPE,universal_newlines=True)

        self.microphone_subscriber = self.create_subscription(msg_type=AudioData, topic="audio", callback=self.audio_stream, qos_profile=10)
        
        self.get_logger().info("started subscribing")
        
        self.kk_monitor_thread = Thread(target=self.monitor_program_output)
        self.kk_monitor_thread.start()
        self.get_logger().info("Started a non-blocking thread")
        
        #self.monitor_program_output()

    def audio_stream(self, msg):
        #self.get_logger().info("sain data: " + str(msg.data))
        
        input_buffer=bytes(msg.data)
             
        self.kiirkirjutaja_process.stdin.write(str(input_buffer))
        
        
        
    # Get constant output of speech synthesis
    def monitor_program_output(self):
        self.get_logger().info("started waiting for kiirkirjutaja output")
        self.recognised_speech_buffer = ""
        last_input_time = time.time()
        self.started_rec  = False
         
        while True:
            # Read a character from stdout of kiirkirjutaja process
            readable,_,_ = select.select([self.kiirkirjutaja_process.stdout], [], [], 0.1)
            
            if readable:
                output =self.kiirkirjutaja_process.stdout.read(1)
                # If find some output
                if output:
                    self.get_logger().info("sain inputiks:" + output)
                    # Add character to string 
                    self.recognised_speech_buffer+=output
                    last_input_time = time.time()
                    if len(self.recognised_speech_buffer)<2:
                        self.started_rec = True
                        self.get_logger().info('started recognition')
                    # If whitespace then log gotten word
                    
                    msg = String()
                    msg.data = self.recognised_speech_buffer.splitlines()[-1]
                    self.debug_publisher.publish(msg)
            if self.started_rec:
                last_input_time = time.time()
                continue
                if time.time()-last_input_time>4:
                    self.get_logger().info('Detected pause, sending service call to llama_node')
                    self.call_LLM_service()
                    output = self.kiirkirjutaja_process.stdout.read(2000)
                    self.get_logger().info('Restarted speech recognition')
                    self.started_rec = False
                    self.recognised_speech_buffer = ""
                    last_input_time = time.time()

    def call_LLM_service(self):
        while not self.LLama_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = Llama.Request()
        request.input_string = self.recognised_speech_buffer
        future = self.LLama_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=180)
        string_to_synthesize = future.result().output_string
        if future.result() is not None:
            self.get_logger().info('Service call successful, string to synthesize:' + string_to_synthesize)
            self.Synthesize(string_to_synthesize)
            
        else:
            self.get_logger().warning('Service call failed')

    def Synthesize(self, string_to_synthesize):
        #Write to file
        with open("/home/semubot/text-to-speech/to_synthesize.txt", "w") as file:
            file.write(string_to_synthesize)
        
        # Activate Anaconda environmenent and run command to synthesize the robots output into a wav file
        activate_cmd = f"/home/semubot/anaconda3/bin/activate {conda_env} && cd /home/semubot/text-to-speech && "
        program_cmd = "/home/semubot/anaconda3/envs/transformer-tts/bin/python synthesizer.py --speaker mari --config config.yaml to_synthesize.txt synthesized.wav"
        full_cmd = f"{activate_cmd}{program_cmd}"
        self.tts = subprocess.run(full_cmd, shell=True, universal_newlines=True)
        self.get_logger().info("Finished synthesizing")
        
        # TEMPORARY, play audio using play command
        play_audio_cmd = "/usr/bin/play /home/semubot/text-to-speech/synthesized.wav"
        self.play_audio = subprocess.run(play_audio_cmd, shell=True, universal_newlines=True)
        

def main(args=None):
    rclpy.init(args=args)
    node = semubotSpeechNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()