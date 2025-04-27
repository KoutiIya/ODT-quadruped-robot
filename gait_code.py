from machine import Pin, I2C, time_pulse_us
import time
import ustruct
import math

# PCA9685 registers
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

# Servo channels mapping
LEFT_FRONT_SHOULDER = 0
LEFT_FRONT_ELBOW = 1
LEFT_BACK_SHOULDER = 2
LEFT_BACK_ELBOW = 3
RIGHT_FRONT_SHOULDER = 4
RIGHT_FRONT_ELBOW = 5
RIGHT_BACK_SHOULDER = 6
RIGHT_BACK_ELBOW = 7

# HC-SR04 Ultrasonic sensor pins
TRIG_PIN = 5
ECHO_PIN = 18

# Movement parameters
STEP_HEIGHT = 30  # How high the leg lifts in degrees
STEP_LENGTH = 40  # How far the leg moves forward/backward in degrees
TURN_ANGLE = 20   # How much to turn in degrees
OBSTACLE_DISTANCE = 10  # Distance in cm to trigger turning

class PCA9685:
    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.reset()
        
    def reset(self):
        self.write_byte_data(MODE1, 0x00)
        
    def write_byte_data(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
        
    def read_byte_data(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
        
    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency in Hz"""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)
        
        oldmode = self.read_byte_data(MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.write_byte_data(MODE1, newmode)  # go to sleep
        self.write_byte_data(PRESCALE, prescale)
        self.write_byte_data(MODE1, oldmode)
        time.sleep_ms(5)
        self.write_byte_data(MODE1, oldmode | 0xa1)  # auto increment
        
    def set_pwm(self, channel, on, off):
        """Sets the PWM pulse for a channel"""
        self.i2c.writeto_mem(self.address, LED0_ON_L + 4 * channel, 
                            ustruct.pack('<HH', on, off))
        
    def set_servo_pulse(self, channel, pulse_width_us):
        """Set servo pulse width in microseconds"""
        value = int(pulse_width_us * 4096 / 20000)  # 20ms period
        self.set_pwm(channel, 0, value)
        
    def set_servo_angle(self, channel, angle):
        """Set servo angle in degrees (0-180)"""
        # Map angle (0-180) to pulse width (500-2500 us)
        pulse = int(500 + (angle / 180) * 2000)
        self.set_servo_pulse(channel, pulse)


class HCSR04:
    """HC-SR04 ultrasonic distance sensor driver."""
    
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=30000):
        """Initialize the sensor with the given trigger and echo pins."""
        self.trigger = Pin(trigger_pin, Pin.OUT, 0)
        self.echo = Pin(echo_pin, Pin.IN)
        self.echo_timeout_us = echo_timeout_us
        
    def distance_cm(self):
        """Return the distance in centimeters."""
        # Trigger the sensor with a 10us pulse
        self.trigger.value(0)
        time.sleep_us(5)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        # Wait for the echo and measure the duration
        duration = time_pulse_us(self.echo, 1, self.echo_timeout_us)
        if duration < 0:
            return None  # Timeout
        
        # Calculate distance: time = distance/speed, speed of sound = 343.2 m/s
        # Distance in cm = duration (us) * speed of sound (cm/us) / 2 (round trip)
        distance = duration * 0.01715  # (343.2 / 10^4) / 2
        return distance


class QuadrupedRobot:
    def __init__(self, pca, ultrasonic):
        self.pca = pca
        self.ultrasonic = ultrasonic
        
        # Current angles of each servo
        self.current_angles = {
            LEFT_FRONT_SHOULDER: 90,
            LEFT_FRONT_ELBOW: 90,
            LEFT_BACK_SHOULDER: 90,
            LEFT_BACK_ELBOW: 90,
            RIGHT_FRONT_SHOULDER: 90,
            RIGHT_FRONT_ELBOW: 90,
            RIGHT_BACK_SHOULDER: 90,
            RIGHT_BACK_ELBOW: 90
        }
        
        # Initialize all servos to their starting positions
        for channel, angle in self.current_angles.items():
            self.pca.set_servo_angle(channel, angle)
            
        # Movement direction (1 for forward, -1 for right turn)
        self.direction = 1
    
    def move_servo_smoothly(self, channel, target_angle, steps=10, delay_ms=20):
        """Move a servo smoothly from current to target angle"""
        start_angle = self.current_angles[channel]
        step_size = (target_angle - start_angle) / steps
        
        for i in range(steps + 1):
            angle = start_angle + step_size * i
            self.pca.set_servo_angle(channel, angle)
            self.current_angles[channel] = angle
            time.sleep_ms(delay_ms)
    
    def check_obstacle(self):
        """Check if there's an obstacle in front and return True if detected"""
        distance = self.ultrasonic.distance_cm()
        if distance is not None and distance < OBSTACLE_DISTANCE:
            print(f"Obstacle detected at {distance:.1f} cm! Turning right...")
            return True
        return False
    
    def turn_right(self):
        """Turn the robot to the right"""
        # Adjust servo angles to turn right
        # Legs diagonal pairs move together
        
        # Step 1: Lift diagonal pair 1 (LF and RB)
        self.move_servo_smoothly(LEFT_FRONT_ELBOW, 90 - STEP_HEIGHT)
        self.move_servo_smoothly(RIGHT_BACK_ELBOW, 90 - STEP_HEIGHT)
        
        # Step 2: Rotate shoulders for diagonal pair 1
        self.move_servo_smoothly(LEFT_FRONT_SHOULDER, 90 + TURN_ANGLE)
        self.move_servo_smoothly(RIGHT_BACK_SHOULDER, 90 - TURN_ANGLE)
        
        # Step 3: Lower diagonal pair 1
        self.move_servo_smoothly(LEFT_FRONT_ELBOW, 90)
        self.move_servo_smoothly(RIGHT_BACK_ELBOW, 90)
        
        # Step 4: Lift diagonal pair 2 (RF and LB)
        self.move_servo_smoothly(RIGHT_FRONT_ELBOW, 90 - STEP_HEIGHT)
        self.move_servo_smoothly(LEFT_BACK_ELBOW, 90 - STEP_HEIGHT)
        
        # Step 5: Rotate shoulders for diagonal pair 2
        self.move_servo_smoothly(RIGHT_FRONT_SHOULDER, 90 + TURN_ANGLE)
        self.move_servo_smoothly(LEFT_BACK_SHOULDER, 90 - TURN_ANGLE)
        
        # Step 6: Lower diagonal pair 2
        self.move_servo_smoothly(RIGHT_FRONT_ELBOW, 90)
        self.move_servo_smoothly(LEFT_BACK_ELBOW, 90)
        
        # Step 7: Return all shoulders to center position
        self.move_servo_smoothly(LEFT_FRONT_SHOULDER, 90)
        self.move_servo_smoothly(RIGHT_BACK_SHOULDER, 90)
        self.move_servo_smoothly(RIGHT_FRONT_SHOULDER, 90)
        self.move_servo_smoothly(LEFT_BACK_SHOULDER, 90)
    
    def walk_gait(self):
        """Execute one cycle of quadruped walking gait"""
        # We use a diagonal gait where diagonal legs move together
        # Phase 1: Move diagonal pair 1 (LF and RB)
        
        # Check for obstacles
        if self.check_obstacle():
            self.turn_right()
            return
        
        # Step 1: Lift diagonal pair 1 (LF and RB)
        self.move_servo_smoothly(LEFT_FRONT_ELBOW, 90 - STEP_HEIGHT)
        self.move_servo_smoothly(RIGHT_BACK_ELBOW, 90 - STEP_HEIGHT)
        
        # Step 2: Move shoulders forward for diagonal pair 1
        self.move_servo_smoothly(LEFT_FRONT_SHOULDER, 90 + STEP_LENGTH/2)
        self.move_servo_smoothly(RIGHT_BACK_SHOULDER, 90 + STEP_LENGTH/2)
        
        # Step 3: Lower diagonal pair 1
        self.move_servo_smoothly(LEFT_FRONT_ELBOW, 90)
        self.move_servo_smoothly(RIGHT_BACK_ELBOW, 90)
        
        # Check for obstacles again
        if self.check_obstacle():
            self.turn_right()
            return
        
        # Phase 2: Move diagonal pair 2 (RF and LB)
        
        # Step 4: Lift diagonal pair 2 (RF and LB)
        self.move_servo_smoothly(RIGHT_FRONT_ELBOW, 90 - STEP_HEIGHT)
        self.move_servo_smoothly(LEFT_BACK_ELBOW, 90 - STEP_HEIGHT)
        
        # Step 5: Move shoulders forward for diagonal pair 2
        self.move_servo_smoothly(RIGHT_FRONT_SHOULDER, 90 + STEP_LENGTH/2)
        self.move_servo_smoothly(LEFT_BACK_SHOULDER, 90 + STEP_LENGTH/2)
        
        # Step 6: Lower diagonal pair 2
        self.move_servo_smoothly(RIGHT_FRONT_ELBOW, 90)
        self.move_servo_smoothly(LEFT_BACK_ELBOW, 90)
        
        # Check for obstacles again
        if self.check_obstacle():
            self.turn_right()
            return
        
        # Phase 3: Move body forward by moving all shoulders backward
        
        # Step 7: Move all shoulders back to initial position
        self.move_servo_smoothly(LEFT_FRONT_SHOULDER, 90 - STEP_LENGTH/2)
        self.move_servo_smoothly(RIGHT_BACK_SHOULDER, 90 - STEP_LENGTH/2)
        self.move_servo_smoothly(RIGHT_FRONT_SHOULDER, 90 - STEP_LENGTH/2)
        self.move_servo_smoothly(LEFT_BACK_SHOULDER, 90 - STEP_LENGTH/2)
        
        # Return to center position
        self.move_servo_smoothly(LEFT_FRONT_SHOULDER, 90)
        self.move_servo_smoothly(RIGHT_BACK_SHOULDER, 90)
        self.move_servo_smoothly(RIGHT_FRONT_SHOULDER, 90)
        self.move_servo_smoothly(LEFT_BACK_SHOULDER, 90)


def main():
    try:
        # Initialize I2C
        i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
        
        # Check if PCA9685 is present
        devices = i2c.scan()
        if PCA9685_ADDRESS not in devices:
            print(f"PCA9685 not found at address 0x{PCA9685_ADDRESS:02x}")
            print(f"Available devices: {[hex(d) for d in devices]}")
            if not devices:
                print("No I2C devices found. Check connections.")
                return
            print(f"Using first available device at address 0x{devices[0]:02x}")
            pca = PCA9685(i2c, devices[0])
        else:
            print(f"PCA9685 found at address 0x{PCA9685_ADDRESS:02x}")
            pca = PCA9685(i2c)
        
        # Set PWM frequency for servos (typical 50Hz)
        pca.set_pwm_freq(50)
        
        # Initialize ultrasonic sensor
        ultrasonic = HCSR04(TRIG_PIN, ECHO_PIN)
        
        # Initialize robot
        robot = QuadrupedRobot(pca, ultrasonic)
        
        # Allow time for servos to initialize
        time.sleep(1)
        
        print("Starting quadruped walking sequence...")
        
        # Main control loop
        while True:
            # Execute one walking cycle
            robot.walk_gait()
            
            # Small delay between cycles
            time.sleep_ms(100)
            
    except KeyboardInterrupt:
        print("Program terminated by user")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Try to clean up and return servos to neutral position
        try:
            for channel in range(8):
                pca.set_servo_angle(channel, 90)
        except:
            pass

if __name__ == "__main__":
    main()
