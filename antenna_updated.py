import math
import time
import RPi.GPIO as GPIO
from pymavlink import mavutil

# ========================
# Servo GPIO Config
# ========================
SERVO_X_PIN = 17  # Horizontal axis (azimuth)
SERVO_Y_PIN = 18  # Vertical axis (elevation)

# Servo angle range
MIN_ANGLE = 0
MAX_ANGLE = 180

# PWM setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_X_PIN, GPIO.OUT)
GPIO.setup(SERVO_Y_PIN, GPIO.OUT)

servo_x = GPIO.PWM(SERVO_X_PIN, 50)  # 50Hz PWM
servo_y = GPIO.PWM(SERVO_Y_PIN, 50)

servo_x.start(0)
servo_y.start(0)

def angle_to_pwm_ms(angle):
    return 1.0 + (angle / 180.0)  # 1ms to 2ms for 0 to 180 degrees

def move_servo(x_angle, y_angle):
    x_angle = max(MIN_ANGLE, min(MAX_ANGLE, x_angle))
    y_angle = max(MIN_ANGLE, min(MAX_ANGLE, y_angle))

    pwm_x = angle_to_pwm_ms(x_angle)
    pwm_y = angle_to_pwm_ms(y_angle)

    duty_x = (pwm_x / 20.0) * 100
    duty_y = (pwm_y / 20.0) * 100

    servo_x.ChangeDutyCycle(duty_x)
    servo_y.ChangeDutyCycle(duty_y)
    time.sleep(0.3)
    servo_x.ChangeDutyCycle(0)
    servo_y.ChangeDutyCycle(0)

# ========================
# Coordinate Functions
# ========================
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi/2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(d_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)
    x = math.sin(d_lambda) * math.cos(φ2)
    y = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(d_lambda)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

def calculate_elevation(distance, alt):
    return math.degrees(math.atan2(alt, distance))

# ========================
# Main Tracking Loop
# ========================
GS_LAT = -35.3632621   # Change this
GS_LON = 149.1652374   # Change this
GS_ALT = 10
GS_HEADING_OFFSET = 0

print("Connecting to MAVLink on 127.0.0.1:14550...")
master = mavutil.mavlink_connection('udp:10.101.101.112:14550')
master.wait_heartbeat()
print("Heartbeat received.")

last_servo_az = None

try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            continue

        drone_lat = msg.lat / 1e7
        drone_lon = msg.lon / 1e7
        drone_alt = msg.relative_alt / 1000.0

        dist = haversine(GS_LAT, GS_LON, drone_lat, drone_lon)
        azimuth = calculate_bearing(GS_LAT, GS_LON, drone_lat, drone_lon)
        elevation = calculate_elevation(dist, drone_alt)

        adjusted_azimuth = (azimuth - GS_HEADING_OFFSET + 360) % 360

        print(f"Vertical: {adjusted_azimuth:.1f}°, Horizontal: {elevation:.1f}°, Dist: {dist:.1f}m")

        new_servo_az = adjusted_azimuth / 2
        new_servo_el = 90 + elevation

        if last_servo_az is not None:
            delta = new_servo_az - last_servo_az
            if abs(delta) > 90:
                if delta > 0:
                    new_servo_az -= 180
                else:
                    new_servo_az += 180
                new_servo_az = max(MIN_ANGLE, min(MAX_ANGLE, new_servo_az))

        move_servo(new_servo_az, new_servo_el)
        last_servo_az = new_servo_az

except KeyboardInterrupt:
    print("Exiting...")

finally:
    servo_x.stop()
    servo_y.stop()
    GPIO.cleanup()

