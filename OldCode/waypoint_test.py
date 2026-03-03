import time
from drone import Drone  # assuming your file is named `drone.py`

# Initialize and connect to drone
drone = Drone()
print("✅ Drone connected")

altitude_var = 10
# Take off to 10 meters
drone.takeoff(altitude=altitude_var)

# Optional: wait to stabilize
time.sleep(5)

# Go to Waypoint A (example coordinates)
waypoint_a = (18.2098535, -67.1395124, altitude_var)  # (lat, lon, alt)
print(f"🛫 Going to Waypoint A: {waypoint_a}")
drone.goto_position(*waypoint_a)

# Give it time to reach
time.sleep(15)

drone.loitering(altitude=altitude_var, seconds=5)

# Go to Waypoint B
waypoint_b = (18.2096682, -67.1395554, altitude_var)  # slightly different coords
print(f"🛬 Going to Waypoint B: {waypoint_b}")
drone.goto_position(*waypoint_b)

# Give it time to reach
time.sleep(15)

drone.loitering(altitude=altitude_var, seconds=5)

waypoint_c = (18.2097828, -67.1397022, altitude_var)  # slightly different coords
print(f"🛬 Going to Waypoint C: {waypoint_c}")
drone.goto_position(*waypoint_c)

time.sleep(15)

drone.loitering(altitude=altitude_var, seconds=5)

# Return to Launch Code
drone.return_to_launch()

time.sleep(40)
# Land
# drone.land()

# Disarm
drone.disarm()
print("🏁 Mission completed. Drone disarmed.")