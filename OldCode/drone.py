from pymavlink import mavutil
import time

class Drone:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600,source_system=255,dialect="ardupilotmega", mavlink1=True)  # Adjust the IP and port as necessary
        print("Waiting for heartbeat...")
        print("Manually waiting for HEARTBEAT...")
        while True:
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
            if msg:
                self.mav.target_system = msg.get_srcSystem()
                self.mav.target_component = msg.get_srcComponent()
                print(f"✅ Got HEARTBEAT from system {self.mav.target_system}, component {self.mav.target_component}")
                break
        self.boot_time = time.time()
        self.geofence_bounds = {}
        self.geofence_active = False

    # Drone Set Up 
    def arm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
            )

    def disarm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    # Geofence
    def set_geofence(self, min_lat, max_lat, min_lon, max_lon, min_alt, max_alt):
        self.geofence_active = True
        self.geofence_bounds = {
            "min_lat": min_lat,
            "max_lat": max_lat,
            "min_lon": min_lon,
            "max_lon": max_lon,
            "min_alt": min_alt,
            "max_alt": max_alt
        }
        print("📐 Geofence set.")

    def disable_geofence(self):
        self.geofence_active = False
        self.geofence_bounds = {}
        print("Geofence disabled")

    # ================ Movement ==============
    def get_position(self):
        print("📡 Waiting for GLOBAL_POSITION_INT...")
        start = time.time()
        while time.time() - start < 30:  # wait up to 10 seconds
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                print(f"📍 Got position: lat={lat}, lon={lon}")
                return lat, lon
            time.sleep(0.1)
        raise TimeoutError("❌ Timed out waiting for GLOBAL_POSITION_INT")

    def goto_position(self, lat, lon, alt):
        #if getattr(self, 'geofence_active', False) and not self.is_within_bounds(lat, lon, alt):
        #    print(f"❌ Target ({lat}, {lon}, {alt}) is outside the geofence.")
        #    self.return_to_launch()
        #    return

        self.set_mode("GUIDED")
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        time_boot_ms = int((time.time() - self.boot_time) * 1000)

        self.mav.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # Use position only
            lat_int, lon_int, alt,
            0, 0, 0,             # Velocity
            0, 0, 0,             # Acceleration
            0, 0                 # Yaw, Yaw rate
        )
        print(f"🛰️ Setpoint sent to: lat={lat}, lon={lon}, alt={alt}")

    def return_to_launch(self):
        print("Returning to launch...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Empty params
        )

    def takeoff(self, altitude=10):
        # Set mode to GUIDED
        self.set_mode("GUIDED")

        # Arm the drone
        self.arm()
        print("Drone armed, waiting 3 seconds...")
        time.sleep(3)

        # Send takeoff command
        print(f"Taking off to {altitude} meters...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,       # Confirmation
            0, 0, 0, 0,  # Empty params
            0, 0,        # lat, lon (ignored if 0)
            altitude     # Altitude
        )

        time.sleep(10)  # Give time to reach altitude

    def loitering(self, altitude=10, seconds = 10):
        # Loiter for 10 seconds
        print("Loitering for 10 seconds...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            0,  # Confirmation
            seconds, # Loiter time (in seconds)
            0, 0, 0,
            0, 0,  # lat/lon (ignored if 0)
            altitude
        )

    def land(self):
        # Land
        print("Initiating landing...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0,  # lat/lon
            0      # alt
        )
        time.sleep(10)

    def set_mode(self, mode):
        mode_id = self.mav.mode_mapping().get(mode)
        if mode_id is None:
            raise Exception(f"Unknown mode: {mode}")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mode set to {mode}")

    def is_within_bounds(self, lat, lon, alt):
        b = self.geofence_bounds
        return (b["min_lat"] <= lat <= b["max_lat"] and
                b["min_lon"] <= lon <= b["max_lon"] and
                b["min_alt"] <= alt <= b["max_alt"])