import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import Llama
from audio_common_msgs.msg import AudioData
import subprocess
from threading import Thread
import time
import select
import numpy as np


# Change these variables if neccessary
conda_env = "transformer-tts"
#kiirkirj_command = "rec -t raw -r 16k -e signed -b 16 -c 1 - | play -t raw -e signed-integer -b 16 -c 1 -r 16k -"
#kiirkirj_command = "play -t raw -e signed-integer -b 16 -c 1 -r 16k -"
kiirkirj_command = "docker exec -i kiirkirjutaja python main.py -"
#kiirkirj_command = "rec -t raw -r 16k -e signed -b 16 -c 1 - | docker exec -i kiirkirjutaja python main.py -"



# For synthesis (assuming conda env is set up)
python_path = "/home/semubot/anaconda3/envs/transformer-tts/bin/python"
tts_path = "/home/semubot/text-to-speech"
conda_path = "/home/semubot/anaconda3/bin/activate" # Path to anaconda activate script
tts_file_path = f"{tts_path}/to_synthesize"

class SemubotSpeechNode(Node):
    def __init__(self):
        super().__init__('SemubotSpeechNode')
        
        self.output_detected = True
        
        # create client to send service request to Llama node
        self.LLama_client = self.create_client(Llama, 'Llama_service')
        
        # Publisher for debug topic
        self.debug_publisher = self.create_publisher(String, 'Speech_topic_debug', 10)
        
        # Start Speech synthesis
        self.start_speech_recogintion(kiirkirj_command)
        
        

    def start_speech_recogintion(self, program_cmd):
        # Start kiirkirjutaja in container
        self.kiirkirjutaja_process = subprocess.Popen(program_cmd, shell=True, stdout=subprocess.PIPE, stdin = subprocess.PIPE)

        # Subscribe to audio stream
        self.microphone_subscriber = self.create_subscription(msg_type=AudioData, topic="audio", callback=self.audio_stream, qos_profile=10)
        self.mic_subscriber_start_time = None
        self.get_logger().info("started subscribing")
        
        self.kk_monitor_thread = Thread(target=self.monitor_program_output)
        self.kk_monitor_thread.start()
        self.get_logger().info("Started a non-blocking thread")
        
        #self.monitor_program_output()

    def audio_stream(self, msg):
        
        input_bytes=bytes(msg.data)
     
        self.kiirkirjutaja_process.stdin.write(input_bytes)
        
    # Get constant output of speech synthesis
    def monitor_program_output(self):
        self.get_logger().info("started waiting for kiirkirjutaja output")
        self.recognised_speech_buffer = bytes()
        last_input_time = time.time()
        self.started_rec  = False
         
        while True:
            # Read a character from stdout of kiirkirjutaja process
            readable,_,_ = select.select([self.kiirkirjutaja_process.stdout], [], [], 0.1)
            
            if readable:
                output = self.kiirkirjutaja_process.stdout.read(1)
                # If find some output
                if output:
                    self.get_logger().info("sain inputiks:" + str(output))
                    # Add character to string 
                    self.recognised_speech_buffer+=output
                    last_input_time = time.time()
                    if len(self.recognised_speech_buffer)<2:
                        self.started_rec = True
                        self.get_logger().info('started recognition')
                    
                    #msg = String()
                    #msg.data = self.recognised_speech_buffer.decode("utf-8").splitlines()[-1]
                    #self.debug_publisher.publish(msg)
            if self.started_rec:
                if time.time()-last_input_time>4:
                    self.get_logger().info('Detected pause, sending service call to llama_node')
                    
                    self.call_LLM_service()
                    #output = self.kiirkirjutaja_process.stdout.read(2000)
                    self.get_logger().info('Restarted speech recognition')
                    self.started_rec = False
                    self.recognised_speech_buffer = bytes()
                    last_input_time = time.time()

    def call_LLM_service(self):
        request = Llama.Request()
        request.input_string = self.recognised_speech_buffer.decode("utf-8")
        self.get_logger().info('Whole recognized text was: ' + request.input_string)
        while not self.LLama_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        future = self.LLama_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=180)
        string_to_synthesize = future.result().output_string
        if future.result() is not None:
            self.get_logger().info('Llama service call successful, string to synthesize:' + string_to_synthesize)
            
            self.Synthesize(string_to_synthesize, voice="mari")
            
        else:
            self.get_logger().warning('Llama service call failed')



    # 
    def Synthesize(self, string_to_synthesize, voice):
        # Write string to file
        with open(f"{tts_file_path}.txt", "w") as file:
            file.write(string_to_synthesize)
        
        # Activate Anaconda environmenent and construct the synthesis command
        activate_cmd = f"{conda_path} {conda_env} && cd {tts_path} && "
        program_cmd = f"{python_path} synthesizer.py --speaker {voice} --config config.yaml {tts_file_path}.txt {tts_file_path}.wav"
        full_cmd = f"{activate_cmd}{program_cmd}"
        
        # Run the synthesis command
        self.tts = subprocess.run(full_cmd, shell=True, universal_newlines=True)
        
        self.get_logger().info("Finished synthesizing")
        
        # TEMPORARY, play audio using play command
        play_audio_cmd = f"/usr/bin/play {tts_file_path}.wav"
        self.play_audio = subprocess.run(play_audio_cmd, shell=True, universal_newlines=True)
        

def main(args=None):
    rclpy.init(args=args)
    node = SemubotSpeechNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()