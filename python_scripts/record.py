import subprocess
import time
import signal
import os

def is_ros2_node_running(node_name):
    """
    Check if a specific ROS 2 node is running by using `ros2 node list`.
    """
    try:
        # Run `ros2 node list` to get the list of active nodes
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, check=True)
        # Check if the node_name is in the output
        return node_name in result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error checking ROS 2 nodes: {e}")
        return False
    
def kill_gazebo_processes():
    try:
        # Find processes with 'gazebo' in their name
        result = subprocess.run(['pgrep', '-f', 'gazebo'], stdout=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print("No Gazebo processes found.")
            return

        # Get list of process IDs
        process_ids = result.stdout.strip().split("\n")
        print(f"Found Gazebo processes: {process_ids}")

        # Kill each process
        for pid in process_ids:
            print(f"Killing process with PID: {pid}")
            subprocess.run(['kill', '-9', pid], check=True)

        print("All Gazebo processes terminated.")
    except Exception as e:
        print(f"Error while killing Gazebo processes: {e}")

def launch_gazebo_with_world(world_number):
    """
    Launch Gazebo with the specified world file using ROS 2 launch.
    """
    try:
        # Run the ROS 2 launch command with a custom world in a new process group
        command = [
            "ros2", "launch", "gazebo_ros", "gazebo.launch.py",
            f"world:={world_number}.world"
        ]
        process = subprocess.Popen(
            command,
            preexec_fn=os.setsid,  # Start the process in a new process group
        )
        print(f"Launching Gazebo with world: {world_number} (PID: {process.pid})")
        return process
    except Exception as e:
        print(f"Error launching Gazebo with world {world_number}: {e}")
        return None

def terminate_process(process):
    """
    Terminate the process group of the given process.
    """
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Terminate the process group
        process.wait()  # Wait for the process to fully terminate
        print(f"Process (PID: {process.pid}) terminated successfully.")
    except Exception as e:
        print(f"Error terminating process (PID: {process.pid}): {e}")

def main():
    #for scene_number in range(10,60):
    for scene_number in range(62,66):
        try:
            print(f"scene_number: {scene_number}")
            gazebo_process = launch_gazebo_with_world(scene_number)
            time.sleep(10) # waiting for gazebo to start
            # Define the shell script to run and the ROS 2 node to monitor
            shell_script = "./build.sh" 
            target_node = "/scene_record"

            # Start the shell script as a subprocess
            shell_process = subprocess.Popen(shell_script, shell=True, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))
            print(f"Started shell script with PID: {shell_process.pid}")

            try:
                
                # Poll until the ROS 2 node is detected
                while not is_ros2_node_running(target_node):
                    print(f"Waiting for ROS 2 node '{target_node}' to start...")
                    time.sleep(1)  # Wait before checking again

                print(f"ROS 2 node '{target_node}' is running. waiting to complete.")

                while is_ros2_node_running(target_node):
                    print(f"Waiting for ROS 2 node '{target_node}' to stop...")
                    time.sleep(5)  # Wait before checking again

                print(f"ROS 2 node '{target_node}' completed. Terminating the shell script.")

                current_folder_name = "/home/prabha/Desktop/recordings/testing/scene"
                new_folder_name = f"/home/prabha/Desktop/recordings/testing/scene{scene_number}"
                os.rename(current_folder_name, new_folder_name)

                # current_folder_name = "/home/prabha/Desktop/recordings/training/scene"
                # new_folder_name = f"/home/prabha/Desktop/recordings/training/scene{scene_number}"
                # os.rename(current_folder_name, new_folder_name)

                # current_folder_name = "/home/prabha/Desktop/recordings/validation/scene"
                # new_folder_name = f"/home/prabha/Desktop/recordings/validation/scene{scene_number}"
                # os.rename(current_folder_name, new_folder_name)

            finally:
                # Terminate the shell script
                shell_process.terminate()
                shell_process.wait()  # Wait for the shell script to terminate
                time.sleep(10)
                print("Shell script terminated.")
        finally:
            terminate_process(gazebo_process)
            kill_gazebo_processes()

if __name__ == "__main__":
    main()
