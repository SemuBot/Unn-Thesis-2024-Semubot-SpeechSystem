import time
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Llama
import subprocess

class ServiceNode(Node):

    def __init__(self):
        super().__init__('service_node')
        # Initial chat
        self.current_chat="<user>\nSa oled sotsiaalne humanoidrobot nimega Semu. Sa oled sõbralik, abivalmis ja viisakas. Sinu ülesanne on vastata küsimustele.\n<assistant>\nVäga meeldiv. Ootan teise inimesega suhtlemist."
        #  Check if docker container running
        llama_container_check = subprocess.run("docker ps", capture_output=True, text=True, shell=True)
        if "llama_cont" not in llama_container_check.stdout.strip():
            # Start docker container
            docker_cmd= "docker run --runtime nvidia -it -d --network host --volume /tmp/argus_socket:/tmp/argus_socket --volume /etc/enctune.conf:/etc/enctune.conf --volume /etc/nv_tegra_release:/etc/nv_tegra_release --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model --volume /var/run/dbus:/var/run/dbus --volume /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket --volume /var/run/docker.sock:/var/run/docker.sock --volume /home/albert/jetson-containers/data:/data --volume /home/albert/llama.cpp/models:/opt/llama.cpp/models --device /dev/snd --device /dev/bus/usb -e DISPLAY=:1 --name llama_cont -v /tmp/.X11-unix/:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -e XAUTHORITY=/tmp/.docker.xauth dustynv/llama_cpp:gguf-r35.3.1"
            # Remove old stopped docker container
            old_docker_container = subprocess.run("docker remove llama_cont", shell=True)
            docker_container = subprocess.run(docker_cmd, shell=True)
            self.llama_cont_active = True
        self.srv = self.create_service(Llama, 'Llama_service', self.callback)

    def callback(self, request, response):
        self.get_logger().info(f"Received: {request.input_string}")

        processed_string = self.preprocess(request.input_string)

        # Add the user input into the chat correctly
        self.current_chat += "\n<user>\n" + processed_string + "\n<assistant>\n"
        
        # Run a program using subprocess
        try:
            llamacpp_cmd = f"docker exec --workdir /opt/llama.cpp/bin llama_cont ./main -m /opt/llama.cpp/models/Llammas_q4_K_M.gguf --prompt \"{self.current_chat}\" --n-predict 256 --ctx-size 300 --batch-size 192 --n-gpu-layers 999 -e"
            output = subprocess.run(llamacpp_cmd, capture_output=True, text=True, shell=True)

            output_string = output.stdout.strip() # Get the output from llama_cpp
            lines = output_string.splitlines() # split lines from llama.cpp output

            self.current_chat = output_string
            curr_line_idx = len(lines)-1
            response.output_string=""

            # find only the relevant lines, starting from the last <assistant> line
            for i,line in enumerate(lines):
                if "<assistant>" in lines[curr_line_idx]:
                    response.output_string = "\n".join(lines[curr_line_idx+1:])
                    break
                else:
                    curr_line_idx-=1
            self.get_logger().info(f"Sent: {response.output_string}")
            return response
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error running subprocess: {e}")
            response.output_string = ""  # Clear the response string
            return response
    
    def stop_llama_container(self):
        if self.llama_cont_active:
            stop_docker = subprocess.run("docker stop llama_cont", shell=True)
            stop_docker = subprocess.run("docker remove llama_cont", shell=True)
            self.get_logger().info(f"Removed llama_cont container")
        else:
            self.get_logger().info(f"Tried to stop llama container, but container was not running")
    
    def preprocess(self, string):
        processed = string.replace('\n-','\n').replace('\n','')
        return processed


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    node.stop_llama_container()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()