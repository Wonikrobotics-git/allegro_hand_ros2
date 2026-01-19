import subprocess
import os
import sys

def run_command(cmd):
    """
    Executes a shell command and raises an exception if an error occurs.
    """
    try:
        # Print the command being executed for debugging
        print(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True, text=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {cmd}")
        print(f"Return code: {e.returncode}")
        raise e

def find_interface_by_path_tag(target_tag):
    """
    Searches all interfaces in /sys/class/net/ to find the device matching the ID_PATH_TAG.
    (It searches the entire directory to find devices that may have already been renamed)
    """
    net_path = "/sys/class/net"
    
    if not os.path.exists(net_path):
        return None

    # Iterate through all network interfaces (excluding loopback 'lo')
    for iface in os.listdir(net_path):
        if iface == "lo": continue
            
        sys_path = os.path.join(net_path, iface)
        
        try:
            # Execute udevadm info (options removed for backward compatibility)
            cmd = ["udevadm", "info", sys_path]
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            
            # Parse output line by line
            for line in result.stdout.splitlines():
                if "ID_PATH_TAG=" in line:
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        current_tag = parts[1].strip()
                        if current_tag == target_tag:
                            return iface
        except:
            continue

    return None

def setup_can_interface(target_tag, new_name, bitrate=1000000):
    """
    Finds the CAN device by tag, renames it, and brings it up.
    """
    print(f"--- Starting CAN device setup (Target: {new_name}) ---")

    # 1. Find current device name
    current_name = find_interface_by_path_tag(target_tag)
    
    if not current_name:
        print(f"[Failed] Could not find device matching tag ({target_tag}).")
        return False

    print(f"Device found: Current name is '{current_name}'.")

    try:
        # 2. Bring interface down (Required before renaming or changing type)
        run_command(f"sudo ip link set {current_name} down")

        # 3. Rename interface
        # Only rename if the current name differs from the target name
        if current_name != new_name:
            print(f"Attempting to rename: {current_name} -> {new_name}")
            run_command(f"sudo ip link set {current_name} name {new_name}")
        else:
            print(f"Name is already {new_name}. Skipping rename.")

        # 4. Set bitrate (Type CAN)
        # Use new_name since the interface has been renamed
        run_command(f"sudo ip link set {new_name} type can bitrate {bitrate}")

        # 5. Bring interface up
        run_command(f"sudo ip link set {new_name} up")
        
        print(f"\n[Success] Device {new_name} is up with bitrate {bitrate}bps.")
        return True

    except Exception as e:
        print(f"\n[Error] An issue occurred during setup: {e}")
        return False

# --- Usage Example ---
if __name__ == "__main__":
    # 1. Tag of the device to find (obtained from udevadm info)
    #   - $ udevadm info /sys/class/net/can* 
    #     ID_PATH_TAG=pci-0000_00_14_0-usb-0_9_1_0 
    MY_TAG = "pci-0000_00_14_0-usb-0_9_1_0"
    
    # 2. Target name and bitrate
    TARGET_NAME = "can_sensor_x"
    TARGET_BITRATE = 1000000
    
    # Run function
    setup_can_interface(MY_TAG, TARGET_NAME, TARGET_BITRATE)