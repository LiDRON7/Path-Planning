from pymavlink import mavutil
import time

#  Connect (using the --net=host shared network)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("📡 Waiting for Heartbeat...")
master.wait_heartbeat()
print("✅ Heartbeat received!")

# Function to send setpoints 
def send_position_target(lat, lon, alt):
    master.mav.set_position_target_global_int_send(
        0,                                   # time_boot_ms (Set to 0 to avoid overflow)
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                  # Bitmask (Position only)
        int(lat * 1e7),                      # Lat converted to integer
        int(lon * 1e7),                      # Lon converted to integer
        float(alt),                          # Altitude
        0, 0, 0,                             # Velocity 
        0, 0, 0,                             # Acceleration 
        0, 0                                 # Yaw 
    )

# Start streaming before arming
# We send the "Home" or "Takeoff" point 20 times to wake up the listener
print("🛰️ Streaming setpoints...")
for i in range(20):
    send_position_target(18.2098535, -67.1395124, 5.0)
    time.sleep(0.1)

#  Switch to OFFBOARD Mode
# PX4 (main_mode=6, sub_mode=0) is usually OFFBOARD
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, 1, 6, 0, 0, 0, 0, 0
)
print("🕹️ Mode set to OFFBOARD")

# Arm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("⚔️ Motors Armed!")

#  Keep Streaming to stay in the air
print("🛫 Lifting off...")
for i in range(100): # Fly for 10 seconds 
    send_position_target(18.2098535, -67.1395124, 5.0)
    time.sleep(0.1)

#  Land
print("🛬 Landing...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)