#!/usr/bin/env python3
"""
AprilTag Autonomous Docking Controller
=========================================
Raspberry Pi Zero - AprilTag detection and control command generation

Hardware: Raspberry Pi Zero with USB camera or CSI camera
Target: tag36h11 AprilTag, 200mm physical size
Detection range: 0.2m to 4.0m
Camera resolution: 320x240 (optimized for Pi Zero performance)

Author: AeroSentinel
"""

import cv2
import numpy as np
import apriltag
import time
import struct
import serial
import argparse
import os
import sys
from collections import deque

# =============================================================================
# CONFIGURATION PARAMETERS - TUNED FOR REAL-WORLD ROVER DOCKING
# =============================================================================

class DockingConfig:
    """Centralized configuration for the docking system"""
    
    # === AprilTag Parameters ===
    TAG_FAMILY = "tag36h11"
    TAG_SIZE = 0.200  # 200mm physical tag size
    
    # === Camera Parameters (adjust for your setup) ===
    # Focal length approximation for 320x240 with typical wide-angle lens
    FOCAL_LENGTH = 250.0  # pixels (estimate, will need calibration)
    CAMERA_CENTER_X = 160.0  # half of 320
    CAMERA_CENTER_Y = 120.0  # half of 240
    
    # === Phase Thresholds ===
    PHASE_B_THRESHOLD = 1.0  # meters - switch to precision mode below this
    
    # === Phase A: Acquisition (> 1.0m) ===
    PHASE_A_MAX_SPEED = 60  # 0-100 scale
    PHASE_A_TURN_GAIN = 0.8  # proportional steering gain
    PHASE_A_DISTANCE_GAIN = 15.0  # speed adjustment based on distance
    
    # === Phase B: Precision (< 1.0m) ===
    PHASE_B_MAX_SPEED = 25  # reduced for precision
    PHASE_B_TURN_GAIN = 1.2  # tighter steering
    PHASE_B_DISTANCE_GAIN = 20.0
    PHASE_B_STOP_DISTANCE = 0.15  # 15cm - final stop threshold
    
    # === Search Behavior ===
    SEARCH_TIMEOUT_MS = 500  # ms without tag before searching
    SEARCH_ROTATE_SPEED = 30  # turn rate during search
    SEARCH_FORWARD_SPEED = 10  # minimal forward during search
    
    # === Docking Complete ===
    DOCKED_DISTANCE = 0.10  # 10cm - consider docked
    
    # === Noise Filtering (EMA) ===
    ERROR_ALPHA = 0.3  # EMA smoothing factor (0-1, lower = smoother)
    DISTANCE_ALPHA = 0.4
    
    # === Safety ===
    COMMAND_TIMEOUT_MS = 1000  # stop if no commands received
    MAX_COMMAND_AGE_MS = 500  # warning threshold
    
    # === Communication ===
    BAUD_RATE = 115200
    COMMAND_RATE_HZ = 15


# =============================================================================
# LOW-PASS FILTER (EMA) FOR NOISE SMOOTHING
# =============================================================================

class EMAFilter:
    """
    Exponential Moving Average filter for smoothing noisy sensor data.
    Helps reduce jitter from AprilTag detection while maintaining responsiveness.
    """
    
    def __init__(self, alpha=0.3):
        """
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive, less smooth
        """
        self.alpha = alpha
        self.value = None
        self.initialized = False
    
    def update(self, new_value):
        """Update filter with new reading, return smoothed value"""
        if not self.initialized:
            self.value = new_value
            self.initialized = True
        else:
            # EMA: value = alpha * new + (1 - alpha) * old
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        """Reset filter state"""
        self.value = None
        self.initialized = False


# =============================================================================
# APRILTAG DETECTOR WRAPPER
# =============================================================================

class AprilTagDetector:
    """
    Wrapper for AprilTag detection with camera handling.
    Optimized for Raspberry Pi Zero performance.
    """
    
    def __init__(self, config):
        self.config = config
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(
                families=config.TAG_FAMILY,
                border=1,
                nthreads=4,
                quad_decimate=1.0,  # No decimation for accuracy
                quad_blur=0.0,
                refine_edges=True,
                refine_decode=True,
                refine_pose=True,
                debug=False
            )
        )
        
        # Camera matrix (will be loaded from calibration if available)
        self.camera_matrix = np.array([
            [config.FOCAL_LENGTH, 0, config.CAMERA_CENTER_X],
            [0, config.FOCAL_LENGTH, config.CAMERA_CENTER_Y],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Distortion coefficients (assume minimal for wide-angle)
        self.dist_coeffs = np.zeros(4)
        
        # Detection stats
        self.last_detection_time = 0
        self.detection_count = 0
        self.consecutive_misses = 0
    
    def detect(self, gray_image, timestamp_ms):
        """
        Detect AprilTags in grayscale image.
        
        Returns:
            dict with keys: 'found', 'center_x', 'center_y', 'distance', 
                           'yaw_error', 'tag_id'
        """
        result = {
            'found': False,
            'center_x': 0,
            'center_y': 0,
            'distance': 0,
            'yaw_error': 0,
            'tag_id': None,
            'timestamp_ms': timestamp_ms
        }
        
        try:
            # Detect tags
            detections = self.detector.detect(gray_image)
            
            if len(detections) > 0:
                # Use the first (closest/largest) tag
                detection = detections[0]
                
                # Get tag ID
                result['tag_id'] = detection['id']
                
                # Get center point (image coordinates)
                center = detection['center']
                result['center_x'] = center[0]
                result['center_y'] = center[1]
                
                # Estimate distance using tag size
                # Tag size in image (average of width/height in pixels)
                tag_width = detection['lb-rb'][0]  # left-bottom to right-bottom
                tag_height = detection['bl-tl'][1]  # bottom-left to top-left
                avg_tag_pixels = (tag_width + tag_height) / 2
                
                if avg_tag_pixels > 0:
                    # Distance = (real_size * focal_length) / pixel_size
                    result['distance'] = (self.config.TAG_SIZE * self.config.FOCAL_LENGTH) / avg_tag_pixels
                
                # Calculate yaw error (horizontal offset from image center)
                # Positive = tag is to the right of center
                result['yaw_error'] = center[0] - self.config.CAMERA_CENTER_X
                
                result['found'] = True
                self.detection_count += 1
                self.last_detection_time = timestamp_ms
                self.consecutive_misses = 0
            else:
                self.consecutive_misses += 1
                
        except Exception as e:
            print(f"Detection error: {e}", file=sys.stderr)
        
        return result


# =============================================================================
# TWO-PHASE DOCKING CONTROLLER
# =============================================================================

class DockingController:
    """
    Two-phase PID-like controller for autonomous docking.
    
    Phase A (Acquisition): Distance > 1.0m
    - Moderate speed, proportional steering
    - Stable long-range behavior
    
    Phase B (Precision): Distance < 1.0m
    - Reduced speed, tighter steering
    - Careful final alignment
    """
    
    def __init__(self, config):
        self.config = config
        
        # State tracking
        self.state = "ACQUISITION"  # ACQUISITION, PRECISION, DOCKED, SEARCH
        self.last_tag_time = 0
        self.docked_start_time = 0
        
        # Filters for smoothing
        self.error_filter = EMAFilter(config.ERROR_ALPHA)
        self.distance_filter = EMAFilter(config.DISTANCE_ALPHA)
        
        # Control outputs
        self.turn_command = 0
        self.speed_command = 0
        self.docked_flag = 0
        
        # Counters
        self.docked_counter = 0
    
    def compute_control(self, detection, current_time_ms):
        """
        Compute control commands based on AprilTag detection.
        
        Returns:
            tuple: (turn, speed, docked_flag, state_name)
        """
        if detection['found']:
            self.last_tag_time = current_time_ms
            
            # Get raw values
            raw_error = detection['yaw_error']
            raw_distance = detection['distance']
            
            # Apply smoothing filters
            smoothed_error = self.error_filter.update(raw_error)
            smoothed_distance = self.distance_filter.update(raw_distance)
            
            # Determine phase based on distance
            if smoothed_distance < self.config.PHASE_B_THRESHOLD:
                self.state = "PRECISION"
            else:
                self.state = "ACQUISITION"
            
            # Compute control based on phase
            if self.state == "ACQUISITION":
                turn, speed = self._compute_acquisition_control(
                    smoothed_error, smoothed_distance
                )
            else:
                turn, speed = self._compute_precision_control(
                    smoothed_error, smoothed_distance
                )
            
            # Check for docked condition
            if smoothed_distance < self.config.DOCKED_DISTANCE:
                self.docked_counter += 1
                if self.docked_counter > 10:  # Confirmed docked
                    self.state = "DOCKED"
                    turn = 0
                    speed = 0
                    self.docked_flag = 1
            else:
                self.docked_counter = 0
                self.docked_flag = 0
                
        else:
            # No tag detected - handle search
            turn, speed, self.state = self._compute_search_control(current_time_ms)
            self.docked_flag = 0
        
        # Clamp outputs to valid ranges
        turn = int(np.clip(turn, -100, 100))
        speed = int(np.clip(speed, 0, 100))
        
        # Store for output
        self.turn_command = turn
        self.speed_command = speed
        
        return turn, speed, self.docked_flag, self.state
    
    def _compute_acquisition_control(self, error, distance):
        """
        Phase A: Long-range acquisition control.
        Moderate speed, stable proportional steering.
        """
        # Turn command: proportional to horizontal error
        # Normalize error based on typical image width
        turn = -error * self.config.PHASE_A_TURN_GAIN / 100.0
        
        # Speed command: based on distance (closer = slower)
        # But maintain minimum speed to keep moving
        distance_factor = max(0.3, min(1.0, distance / 3.0))
        speed = self.config.PHASE_A_MAX_SPEED * distance_factor
        
        # Adjust speed based on turn severity (slow down on sharp turns)
        turn_severity = abs(turn) / 100.0
        speed *= (1.0 - turn_severity * 0.5)
        
        return turn, speed
    
    def _compute_precision_control(self, error, distance):
        """
        Phase B: Precision docking control.
        Reduced speed, tighter steering, careful approach.
        """
        # Turn command: tighter gain for precise alignment
        turn = -error * self.config.PHASE_B_TURN_GAIN / 100.0
        
        # Speed command: much slower for precision
        # Scale with distance for smooth deceleration
        distance_factor = max(0.2, distance / self.config.PHASE_B_THRESHOLD)
        speed = self.config.PHASE_B_MAX_SPEED * distance_factor
        
        # Extra slowdown on turns
        turn_severity = abs(turn) / 100.0
        speed *= (1.0 - turn_severity * 0.6)
        
        # Stop if very close and reasonably aligned
        if distance < self.config.PHASE_B_STOP_DISTANCE:
            speed = 0
            turn = 0
        
        return turn, speed
    
    def _compute_search_control(self, current_time_ms):
        """
        Search behavior when tag is lost.
        Slowly rotate to find the tag again.
        """
        time_since_last = current_time_ms - self.last_tag_time
        
        if time_since_last > self.config.SEARCH_TIMEOUT_MS:
            # Enter search mode - rotate in place
            self.state = "SEARCH"
            
            # Alternate rotation direction based on time
            search_phase = (current_time_ms // 2000) % 2
            turn = self.config.SEARCH_ROTATE_SPEED if search_phase == 0 else -self.config.SEARCH_ROTATE_SPEED
            speed = self.config.SEARCH_FORWARD_SPEED
            
            return turn, speed, "SEARCH"
        
        # Tag just lost, maintain last command briefly
        return self.turn_command, 0, "LOST"
    
    def reset(self):
        """Reset controller state"""
        self.state = "ACQUISITION"
        self.error_filter.reset()
        self.distance_filter.reset()
        self.docked_counter = 0


# =============================================================================
# COMMAND PROTOCOL - SERIAL COMMUNICATION TO ESP32-S3
# =============================================================================

class CommandProtocol:
    """
    Compact command protocol for ESP32-S3 communication.
    
    Packet format (4 bytes):
        [0] turn:    int8_t  -100 to +100
        [1] speed:   uint8_t 0 to 100
        [2] docked:  uint8_t 0 or 1
        [3] checksum: uint8_t XOR of bytes 0-2
    
    Sends at ~15 Hz with failsafe timeout.
    """
    
    PROTOCOL_HEADER = 0xAA
    PROTOCOL_TAIL = 0x55
    
    def __init__(self, config):
        self.config = config
        self.last_send_time = 0
        self.commands_sent = 0
        self.commands_failed = 0
    
    def build_command(self, turn, speed, docked):
        """
        Build command packet bytes.
        
        Args:
            turn:   int    -100 to +100
            speed:  int    0 to 100
            docked: int    0 or 1
        
        Returns:
            bytes: 4-byte command packet
        """
        # Clamp values
        turn = int(np.clip(turn, -100, 100))
        speed = int(np.clip(speed, 0, 100))
        docked = 1 if docked else 0
        
        # Pack into bytes
        turn_byte = turn & 0xFF  # Convert to signed byte
        speed_byte = speed & 0xFF
        docked_byte = docked & 0xFF
        
        # Checksum: XOR of all data bytes
        checksum = turn_byte ^ speed_byte ^ docked_byte
        
        # Build packet: HEADER + turn + speed + docked + checksum + TAIL
        packet = bytes([
            self.PROTOCOL_HEADER,
            turn_byte,
            speed_byte,
            docked_byte,
            checksum,
            self.PROTOCOL_TAIL
        ])
        
        return packet
    
    def send_command(self, serial_port, turn, speed, docked):
        """
        Send command to ESP32-S3 via serial.
        
        Returns:
            bool: True if send successful
        """
        try:
            packet = self.build_command(turn, speed, docked)
            serial_port.write(packet)
            serial_port.flush()
            self.last_send_time = time.time()
            self.commands_sent += 1
            return True
        except Exception as e:
            self.commands_failed += 1
            print(f"Serial send error: {e}", file=sys.stderr)
            return False


# =============================================================================
# MAIN DOCKING SYSTEM
# =============================================================================

class AprilTagDockingSystem:
    """
    Complete AprilTag autonomous docking system.
    Integrates detection, control, and communication.
    """
    
    def __init__(self, config_path=None, serial_device=None):
        # Load configuration
        self.config = DockingConfig()
        
        # Override serial device if provided
        if serial_device:
            self.serial_device = serial_device
        else:
            # Default serial device (Linux/Windows)
            self.serial_device = self._detect_serial_device()
        
        # Initialize components
        self.detector = AprilTagDetector(self.config)
        self.controller = DockingController(self.config)
        self.protocol = CommandProtocol(self.config)
        
        # Serial port
        self.serial_port = None
        
        # Timing
        self.running = False
        self.frame_count = 0
        self.start_time = 0
        
        # Performance metrics
        self.fps = 0
        self.last_fps_update = 0
        self.frame_times = deque(maxlen=30)
    
    def _detect_serial_device(self):
        """Auto-detect ESP32-S3 serial device"""
        import glob
        
        # Common patterns for USB serial devices
        patterns = [
            '/dev/ttyUSB*',      # Linux USB serial
            '/dev/ttyACM*',      # Linux ACM (Arduino)
            '/dev/tty.SLAB*',    # Mac Silicon
            '/dev/tty.usbserial*',  # Mac USB
            'COM*',              # Windows
        ]
        
        for pattern in patterns:
            if pattern.startswith('COM'):
                # Windows - check COM ports 1-10
                import glob as g
                for i in range(1, 11):
                    device = f"COM{i}"
                    if self._test_serial_device(device):
                        return device
            else:
                # Unix-like
                devices = glob.glob(pattern)
                if devices:
                    # Return first available, sorted
                    devices.sort()
                    for dev in devices:
                        if self._test_serial_device(dev):
                            return dev
        
        # Default fallback
        return '/dev/ttyUSB0' if os.name != 'nt' else 'COM3'
    
    def _test_serial_device(self, device):
        """Test if a serial device is accessible"""
        try:
            test_serial = serial.Serial(device, self.config.BAUD_RATE, timeout=0.1)
            test_serial.close()
            return True
        except:
            return False
    
    def initialize(self):
        """Initialize camera and serial connection"""
        # Initialize camera
        print("Initializing camera...")
        self.camera = cv2.VideoCapture(0)
        
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open camera")
        
        # Set camera resolution (optimized for Pi Zero)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.camera.set(cv2.CAP_PROP_FPS, 15)
        
        # Allow camera to stabilize
        time.sleep(0.5)
        
        # Test capture
        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Camera read failed")
        
        # Initialize serial
        print(f"Connecting to ESP32-S3 on {self.serial_device}...")
        try:
            self.serial_port = serial.Serial(
                self.serial_device,
                self.config.BAUD_RATE,
                timeout=0.1,
                write_timeout=0.1
            )
            # Clear buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.5)
            print("Serial connection established")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            print("Running in simulation mode (commands will print only)")
            self.serial_port = None
        
        self.running = True
        self.start_time = time.time()
        
        print("Docking system initialized")
        print(f"  Tag family: {self.config.TAG_FAMILY}")
        print(f"  Tag size: {self.config.TAG_SIZE}m")
        print(f"  Target FPS: {self.config.COMMAND_RATE_HZ}")
        print(f"  Serial device: {self.serial_device}")
    
    def process_frame(self):
        """Process single frame, returns control command"""
        frame_start = time.time()
        
        # Capture frame
        ret, frame = self.camera.read()
        if not ret:
            return None
        
        self.frame_count += 1
        current_time = time.time()
        current_time_ms = int(current_time * 1000)
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTag
        detection = self.detector.detect(gray, current_time_ms)
        
        # Compute control
        turn, speed, docked, state = self.controller.compute_control(
            detection, current_time_ms
        )
        
        # Calculate FPS
        self.frame_times.append(current_time)
        if current_time - self.last_fps_update > 1.0:
            self.fps = len(self.frame_times) / (self.frame_times[-1] - self.frame_times[0])
            self.last_fps_update = current_time
        
        # Send command
        if self.serial_port:
            self.protocol.send_command(self.serial_port, turn, speed, docked)
        
        # Build result
        result = {
            'frame': frame,
            'detection': detection,
            'turn': turn,
            'speed': speed,
            'docked': docked,
            'state': state,
            'fps': self.fps,
            'timestamp_ms': current_time_ms
        }
        
        return result
    
    def run(self):
        """Main loop - process frames and send commands"""
        self.initialize()
        
        print("\n=== DOCKING SYSTEM RUNNING ===")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                result = self.process_frame()
                
                if result:
                    # Print status
                    if self.frame_count % 15 == 0:  # Every ~1 second
                        self._print_status(result)
                
                # Maintain target frame rate
                elapsed = time.time() - self.start_time
                target_interval = 1.0 / self.config.COMMAND_RATE_HZ
                sleep_time = target_interval - elapsed % target_interval
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        finally:
            self.shutdown()
    
    def _print_status(self, result):
        """Print current status"""
        det = result['detection']
        state = result['state']
        
        if det['found']:
            print(f"STATE: {state:10s} | "
                  f"DIST: {det['distance']:.2f}m | "
                  f"ERR: {det['yaw_error']:6.1f}px | "
                  f"TURN: {result['turn']:4d} | "
                  f"SPEED: {result['speed']:3d} | "
                  f"DOCKED: {result['docked']} | "
                  f"FPS: {result['fps']:.1f}")
        else:
            print(f"STATE: {state:10s} | "
                  f"NO TAG DETECTED | "
                  f"TURN: {result['turn']:4d} | "
                  f"SPEED: {result['speed']:3d} | "
                  f"FPS: {result['fps']:.1f}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        
        # Send stop command
        if self.serial_port:
            self.protocol.send_command(self.serial_port, 0, 0, 0)
            time.sleep(0.1)
            self.serial_port.close()
        
        # Release camera
        if hasattr(self, 'camera') and self.camera:
            self.camera.release()
        
        print(f"\nShutdown complete")
        print(f"Frames processed: {self.frame_count}")
        print(f"Commands sent: {self.protocol.commands_sent}")
        print(f"Commands failed: {self.protocol.commands_failed}")


# =============================================================================
# STANDALONE TEST/DEMO MODE
# =============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AprilTag Autonomous Docking System')
    parser.add_argument('--device', '-d', default=None, 
                        help='Serial device for ESP32-S3')
    parser.add_argument('--simulate', '-s', action='store_true',
                        help='Run in simulation mode (no serial)')
    args = parser.parse_args()
    
    # Create system
    system = AprilTagDockingSystem(serial_device=args.device)
    
    # Run
    if args.simulate:
        print("=== SIMULATION MODE ===")
        print("No commands will be sent to rover\n")
        system.serial_port = None
    
    system.run()
"""
AprilTag Autonomous Docking Controller
=========================================
Raspberry Pi Zero - AprilTag detection and control command generation

Hardware: Raspberry Pi Zero with USB camera or CSI camera
Target: tag36h11 AprilTag, 200mm physical size
Detection range: 0.2m to 4.0m
Camera resolution: 320x240 (optimized for Pi Zero performance)

Author: AeroSentinel
"""

import cv2
import numpy as np
import apriltag
import time
import struct
import serial
import argparse
import os
import sys
from collections import deque

# =============================================================================
# CONFIGURATION PARAMETERS - TUNED FOR REAL-WORLD ROVER DOCKING
# =============================================================================

class DockingConfig:
    """Centralized configuration for the docking system"""
    
    # === AprilTag Parameters ===
    TAG_FAMILY = "tag36h11"
    TAG_SIZE = 0.200  # 200mm physical tag size
    
    # === Camera Parameters (adjust for your setup) ===
    # Focal length approximation for 320x240 with typical wide-angle lens
    FOCAL_LENGTH = 250.0  # pixels (estimate, will need calibration)
    CAMERA_CENTER_X = 160.0  # half of 320
    CAMERA_CENTER_Y = 120.0  # half of 240
    
    # === Phase Thresholds ===
    PHASE_B_THRESHOLD = 1.0  # meters - switch to precision mode below this
    
    # === Phase A: Acquisition (> 1.0m) ===
    PHASE_A_MAX_SPEED = 60  # 0-100 scale
    PHASE_A_TURN_GAIN = 0.8  # proportional steering gain
    PHASE_A_DISTANCE_GAIN = 15.0  # speed adjustment based on distance
    
    # === Phase B: Precision (< 1.0m) ===
    PHASE_B_MAX_SPEED = 25  # reduced for precision
    PHASE_B_TURN_GAIN = 1.2  # tighter steering
    PHASE_B_DISTANCE_GAIN = 20.0
    PHASE_B_STOP_DISTANCE = 0.15  # 15cm - final stop threshold
    
    # === Search Behavior ===
    SEARCH_TIMEOUT_MS = 500  # ms without tag before searching
    SEARCH_ROTATE_SPEED = 30  # turn rate during search
    SEARCH_FORWARD_SPEED = 10  # minimal forward during search
    
    # === Docking Complete ===
    DOCKED_DISTANCE = 0.10  # 10cm - consider docked
    
    # === Noise Filtering (EMA) ===
    ERROR_ALPHA = 0.3  # EMA smoothing factor (0-1, lower = smoother)
    DISTANCE_ALPHA = 0.4
    
    # === Safety ===
    COMMAND_TIMEOUT_MS = 1000  # stop if no commands received
    MAX_COMMAND_AGE_MS = 500  # warning threshold
    
    # === Communication ===
    BAUD_RATE = 115200
    COMMAND_RATE_HZ = 15


# =============================================================================
# LOW-PASS FILTER (EMA) FOR NOISE SMOOTHING
# =============================================================================

class EMAFilter:
    """
    Exponential Moving Average filter for smoothing noisy sensor data.
    Helps reduce jitter from AprilTag detection while maintaining responsiveness.
    """
    
    def __init__(self, alpha=0.3):
        """
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive, less smooth
        """
        self.alpha = alpha
        self.value = None
        self.initialized = False
    
    def update(self, new_value):
        """Update filter with new reading, return smoothed value"""
        if not self.initialized:
            self.value = new_value
            self.initialized = True
        else:
            # EMA: value = alpha * new + (1 - alpha) * old
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        """Reset filter state"""
        self.value = None
        self.initialized = False


# =============================================================================
# APRILTAG DETECTOR WRAPPER
# =============================================================================

class AprilTagDetector:
    """
    Wrapper for AprilTag detection with camera handling.
    Optimized for Raspberry Pi Zero performance.
    """
    
    def __init__(self, config):
        self.config = config
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(
                families=config.TAG_FAMILY,
                border=1,
                nthreads=4,
                quad_decimate=1.0,  # No decimation for accuracy
                quad_blur=0.0,
                refine_edges=True,
                refine_decode=True,
                refine_pose=True,
                debug=False
            )
        )
        
        # Camera matrix (will be loaded from calibration if available)
        self.camera_matrix = np.array([
            [config.FOCAL_LENGTH, 0, config.CAMERA_CENTER_X],
            [0, config.FOCAL_LENGTH, config.CAMERA_CENTER_Y],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Distortion coefficients (assume minimal for wide-angle)
        self.dist_coeffs = np.zeros(4)
        
        # Detection stats
        self.last_detection_time = 0
        self.detection_count = 0
        self.consecutive_misses = 0
    
    def detect(self, gray_image, timestamp_ms):
        """
        Detect AprilTags in grayscale image.
        
        Returns:
            dict with keys: 'found', 'center_x', 'center_y', 'distance', 
                           'yaw_error', 'tag_id'
        """
        result = {
            'found': False,
            'center_x': 0,
            'center_y': 0,
            'distance': 0,
            'yaw_error': 0,
            'tag_id': None,
            'timestamp_ms': timestamp_ms
        }
        
        try:
            # Detect tags
            detections = self.detector.detect(gray_image)
            
            if len(detections) > 0:
                # Use the first (closest/largest) tag
                detection = detections[0]
                
                # Get tag ID
                result['tag_id'] = detection['id']
                
                # Get center point (image coordinates)
                center = detection['center']
                result['center_x'] = center[0]
                result['center_y'] = center[1]
                
                # Estimate distance using tag size
                # Tag size in image (average of width/height in pixels)
                tag_width = detection['lb-rb'][0]  # left-bottom to right-bottom
                tag_height = detection['bl-tl'][1]  # bottom-left to top-left
                avg_tag_pixels = (tag_width + tag_height) / 2
                
                if avg_tag_pixels > 0:
                    # Distance = (real_size * focal_length) / pixel_size
                    result['distance'] = (self.config.TAG_SIZE * self.config.FOCAL_LENGTH) / avg_tag_pixels
                
                # Calculate yaw error (horizontal offset from image center)
                # Positive = tag is to the right of center
                result['yaw_error'] = center[0] - self.config.CAMERA_CENTER_X
                
                result['found'] = True
                self.detection_count += 1
                self.last_detection_time = timestamp_ms
                self.consecutive_misses = 0
            else:
                self.consecutive_misses += 1
                
        except Exception as e:
            print(f"Detection error: {e}", file=sys.stderr)
        
        return result


# =============================================================================
# TWO-PHASE DOCKING CONTROLLER
# =============================================================================

class DockingController:
    """
    Two-phase PID-like controller for autonomous docking.
    
    Phase A (Acquisition): Distance > 1.0m
    - Moderate speed, proportional steering
    - Stable long-range behavior
    
    Phase B (Precision): Distance < 1.0m
    - Reduced speed, tighter steering
    - Careful final alignment
    """
    
    def __init__(self, config):
        self.config = config
        
        # State tracking
        self.state = "ACQUISITION"  # ACQUISITION, PRECISION, DOCKED, SEARCH
        self.last_tag_time = 0
        self.docked_start_time = 0
        
        # Filters for smoothing
        self.error_filter = EMAFilter(config.ERROR_ALPHA)
        self.distance_filter = EMAFilter(config.DISTANCE_ALPHA)
        
        # Control outputs
        self.turn_command = 0
        self.speed_command = 0
        self.docked_flag = 0
        
        # Counters
        self.docked_counter = 0
    
    def compute_control(self, detection, current_time_ms):
        """
        Compute control commands based on AprilTag detection.
        
        Returns:
            tuple: (turn, speed, docked_flag, state_name)
        """
        if detection['found']:
            self.last_tag_time = current_time_ms
            
            # Get raw values
            raw_error = detection['yaw_error']
            raw_distance = detection['distance']
            
            # Apply smoothing filters
            smoothed_error = self.error_filter.update(raw_error)
            smoothed_distance = self.distance_filter.update(raw_distance)
            
            # Determine phase based on distance
            if smoothed_distance < self.config.PHASE_B_THRESHOLD:
                self.state = "PRECISION"
            else:
                self.state = "ACQUISITION"
            
            # Compute control based on phase
            if self.state == "ACQUISITION":
                turn, speed = self._compute_acquisition_control(
                    smoothed_error, smoothed_distance
                )
            else:
                turn, speed = self._compute_precision_control(
                    smoothed_error, smoothed_distance
                )
            
            # Check for docked condition
            if smoothed_distance < self.config.DOCKED_DISTANCE:
                self.docked_counter += 1
                if self.docked_counter > 10:  # Confirmed docked
                    self.state = "DOCKED"
                    turn = 0
                    speed = 0
                    self.docked_flag = 1
            else:
                self.docked_counter = 0
                self.docked_flag = 0
                
        else:
            # No tag detected - handle search
            turn, speed, self.state = self._compute_search_control(current_time_ms)
            self.docked_flag = 0
        
        # Clamp outputs to valid ranges
        turn = int(np.clip(turn, -100, 100))
        speed = int(np.clip(speed, 0, 100))
        
        # Store for output
        self.turn_command = turn
        self.speed_command = speed
        
        return turn, speed, self.docked_flag, self.state
    
    def _compute_acquisition_control(self, error, distance):
        """
        Phase A: Long-range acquisition control.
        Moderate speed, stable proportional steering.
        """
        # Turn command: proportional to horizontal error
        # Normalize error based on typical image width
        turn = -error * self.config.PHASE_A_TURN_GAIN / 100.0
        
        # Speed command: based on distance (closer = slower)
        # But maintain minimum speed to keep moving
        distance_factor = max(0.3, min(1.0, distance / 3.0))
        speed = self.config.PHASE_A_MAX_SPEED * distance_factor
        
        # Adjust speed based on turn severity (slow down on sharp turns)
        turn_severity = abs(turn) / 100.0
        speed *= (1.0 - turn_severity * 0.5)
        
        return turn, speed
    
    def _compute_precision_control(self, error, distance):
        """
        Phase B: Precision docking control.
        Reduced speed, tighter steering, careful approach.
        """
        # Turn command: tighter gain for precise alignment
        turn = -error * self.config.PHASE_B_TURN_GAIN / 100.0
        
        # Speed command: much slower for precision
        # Scale with distance for smooth deceleration
        distance_factor = max(0.2, distance / self.config.PHASE_B_THRESHOLD)
        speed = self.config.PHASE_B_MAX_SPEED * distance_factor
        
        # Extra slowdown on turns
        turn_severity = abs(turn) / 100.0
        speed *= (1.0 - turn_severity * 0.6)
        
        # Stop if very close and reasonably aligned
        if distance < self.config.PHASE_B_STOP_DISTANCE:
            speed = 0
            turn = 0
        
        return turn, speed
    
    def _compute_search_control(self, current_time_ms):
        """
        Search behavior when tag is lost.
        Slowly rotate to find the tag again.
        """
        time_since_last = current_time_ms - self.last_tag_time
        
        if time_since_last > self.config.SEARCH_TIMEOUT_MS:
            # Enter search mode - rotate in place
            self.state = "SEARCH"
            
            # Alternate rotation direction based on time
            search_phase = (current_time_ms // 2000) % 2
            turn = self.config.SEARCH_ROTATE_SPEED if search_phase == 0 else -self.config.SEARCH_ROTATE_SPEED
            speed = self.config.SEARCH_FORWARD_SPEED
            
            return turn, speed, "SEARCH"
        
        # Tag just lost, maintain last command briefly
        return self.turn_command, 0, "LOST"
    
    def reset(self):
        """Reset controller state"""
        self.state = "ACQUISITION"
        self.error_filter.reset()
        self.distance_filter.reset()
        self.docked_counter = 0


# =============================================================================
# COMMAND PROTOCOL - SERIAL COMMUNICATION TO ESP32-S3
# =============================================================================

class CommandProtocol:
    """
    Compact command protocol for ESP32-S3 communication.
    
    Packet format (4 bytes):
        [0] turn:    int8_t  -100 to +100
        [1] speed:   uint8_t 0 to 100
        [2] docked:  uint8_t 0 or 1
        [3] checksum: uint8_t XOR of bytes 0-2
    
    Sends at ~15 Hz with failsafe timeout.
    """
    
    PROTOCOL_HEADER = 0xAA
    PROTOCOL_TAIL = 0x55
    
    def __init__(self, config):
        self.config = config
        self.last_send_time = 0
        self.commands_sent = 0
        self.commands_failed = 0
    
    def build_command(self, turn, speed, docked):
        """
        Build command packet bytes.
        
        Args:
            turn:   int    -100 to +100
            speed:  int    0 to 100
            docked: int    0 or 1
        
        Returns:
            bytes: 4-byte command packet
        """
        # Clamp values
        turn = int(np.clip(turn, -100, 100))
        speed = int(np.clip(speed, 0, 100))
        docked = 1 if docked else 0
        
        # Pack into bytes
        turn_byte = turn & 0xFF  # Convert to signed byte
        speed_byte = speed & 0xFF
        docked_byte = docked & 0xFF
        
        # Checksum: XOR of all data bytes
        checksum = turn_byte ^ speed_byte ^ docked_byte
        
        # Build packet: HEADER + turn + speed + docked + checksum + TAIL
        packet = bytes([
            self.PROTOCOL_HEADER,
            turn_byte,
            speed_byte,
            docked_byte,
            checksum,
            self.PROTOCOL_TAIL
        ])
        
        return packet
    
    def send_command(self, serial_port, turn, speed, docked):
        """
        Send command to ESP32-S3 via serial.
        
        Returns:
            bool: True if send successful
        """
        try:
            packet = self.build_command(turn, speed, docked)
            serial_port.write(packet)
            serial_port.flush()
            self.last_send_time = time.time()
            self.commands_sent += 1
            return True
        except Exception as e:
            self.commands_failed += 1
            print(f"Serial send error: {e}", file=sys.stderr)
            return False


# =============================================================================
# MAIN DOCKING SYSTEM
# =============================================================================

class AprilTagDockingSystem:
    """
    Complete AprilTag autonomous docking system.
    Integrates detection, control, and communication.
    """
    
    def __init__(self, config_path=None, serial_device=None):
        # Load configuration
        self.config = DockingConfig()
        
        # Override serial device if provided
        if serial_device:
            self.serial_device = serial_device
        else:
            # Default serial device (Linux/Windows)
            self.serial_device = self._detect_serial_device()
        
        # Initialize components
        self.detector = AprilTagDetector(self.config)
        self.controller = DockingController(self.config)
        self.protocol = CommandProtocol(self.config)
        
        # Serial port
        self.serial_port = None
        
        # Timing
        self.running = False
        self.frame_count = 0
        self.start_time = 0
        
        # Performance metrics
        self.fps = 0
        self.last_fps_update = 0
        self.frame_times = deque(maxlen=30)
    
    def _detect_serial_device(self):
        """Auto-detect ESP32-S3 serial device"""
        import glob
        
        # Common patterns for USB serial devices
        patterns = [
            '/dev/ttyUSB*',      # Linux USB serial
            '/dev/ttyACM*',      # Linux ACM (Arduino)
            '/dev/tty.SLAB*',    # Mac Silicon
            '/dev/tty.usbserial*',  # Mac USB
            'COM*',              # Windows
        ]
        
        for pattern in patterns:
            if pattern.startswith('COM'):
                # Windows - check COM ports 1-10
                import glob as g
                for i in range(1, 11):
                    device = f"COM{i}"
                    if self._test_serial_device(device):
                        return device
            else:
                # Unix-like
                devices = glob.glob(pattern)
                if devices:
                    # Return first available, sorted
                    devices.sort()
                    for dev in devices:
                        if self._test_serial_device(dev):
                            return dev
        
        # Default fallback
        return '/dev/ttyUSB0' if os.name != 'nt' else 'COM3'
    
    def _test_serial_device(self, device):
        """Test if a serial device is accessible"""
        try:
            test_serial = serial.Serial(device, self.config.BAUD_RATE, timeout=0.1)
            test_serial.close()
            return True
        except:
            return False
    
    def initialize(self):
        """Initialize camera and serial connection"""
        # Initialize camera
        print("Initializing camera...")
        self.camera = cv2.VideoCapture(0)
        
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open camera")
        
        # Set camera resolution (optimized for Pi Zero)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.camera.set(cv2.CAP_PROP_FPS, 15)
        
        # Allow camera to stabilize
        time.sleep(0.5)
        
        # Test capture
        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Camera read failed")
        
        # Initialize serial
        print(f"Connecting to ESP32-S3 on {self.serial_device}...")
        try:
            self.serial_port = serial.Serial(
                self.serial_device,
                self.config.BAUD_RATE,
                timeout=0.1,
                write_timeout=0.1
            )
            # Clear buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.5)
            print("Serial connection established")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            print("Running in simulation mode (commands will print only)")
            self.serial_port = None
        
        self.running = True
        self.start_time = time.time()
        
        print("Docking system initialized")
        print(f"  Tag family: {self.config.TAG_FAMILY}")
        print(f"  Tag size: {self.config.TAG_SIZE}m")
        print(f"  Target FPS: {self.config.COMMAND_RATE_HZ}")
        print(f"  Serial device: {self.serial_device}")
    
    def process_frame(self):
        """Process single frame, returns control command"""
        frame_start = time.time()
        
        # Capture frame
        ret, frame = self.camera.read()
        if not ret:
            return None
        
        self.frame_count += 1
        current_time = time.time()
        current_time_ms = int(current_time * 1000)
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTag
        detection = self.detector.detect(gray, current_time_ms)
        
        # Compute control
        turn, speed, docked, state = self.controller.compute_control(
            detection, current_time_ms
        )
        
        # Calculate FPS
        self.frame_times.append(current_time)
        if current_time - self.last_fps_update > 1.0:
            self.fps = len(self.frame_times) / (self.frame_times[-1] - self.frame_times[0])
            self.last_fps_update = current_time
        
        # Send command
        if self.serial_port:
            self.protocol.send_command(self.serial_port, turn, speed, docked)
        
        # Build result
        result = {
            'frame': frame,
            'detection': detection,
            'turn': turn,
            'speed': speed,
            'docked': docked,
            'state': state,
            'fps': self.fps,
            'timestamp_ms': current_time_ms
        }
        
        return result
    
    def run(self):
        """Main loop - process frames and send commands"""
        self.initialize()
        
        print("\n=== DOCKING SYSTEM RUNNING ===")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                result = self.process_frame()
                
                if result:
                    # Print status
                    if self.frame_count % 15 == 0:  # Every ~1 second
                        self._print_status(result)
                
                # Maintain target frame rate
                elapsed = time.time() - self.start_time
                target_interval = 1.0 / self.config.COMMAND_RATE_HZ
                sleep_time = target_interval - elapsed % target_interval
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        finally:
            self.shutdown()
    
    def _print_status(self, result):
        """Print current status"""
        det = result['detection']
        state = result['state']
        
        if det['found']:
            print(f"STATE: {state:10s} | "
                  f"DIST: {det['distance']:.2f}m | "
                  f"ERR: {det['yaw_error']:6.1f}px | "
                  f"TURN: {result['turn']:4d} | "
                  f"SPEED: {result['speed']:3d} | "
                  f"DOCKED: {result['docked']} | "
                  f"FPS: {result['fps']:.1f}")
        else:
            print(f"STATE: {state:10s} | "
                  f"NO TAG DETECTED | "
                  f"TURN: {result['turn']:4d} | "
                  f"SPEED: {result['speed']:3d} | "
                  f"FPS: {result['fps']:.1f}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        
        # Send stop command
        if self.serial_port:
            self.protocol.send_command(self.serial_port, 0, 0, 0)
            time.sleep(0.1)
            self.serial_port.close()
        
        # Release camera
        if hasattr(self, 'camera') and self.camera:
            self.camera.release()
        
        print(f"\nShutdown complete")
        print(f"Frames processed: {self.frame_count}")
        print(f"Commands sent: {self.protocol.commands_sent}")
        print(f"Commands failed: {self.protocol.commands_failed}")


# =============================================================================
# STANDALONE TEST/DEMO MODE
# =============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AprilTag Autonomous Docking System')
    parser.add_argument('--device', '-d', default=None, 
                        help='Serial device for ESP32-S3')
    parser.add_argument('--simulate', '-s', action='store_true',
                        help='Run in simulation mode (no serial)')
    args = parser.parse_args()
    
    # Create system
    system = AprilTagDockingSystem(serial_device=args.device)
    
    # Run
    if args.simulate:
        print("=== SIMULATION MODE ===")
        print("No commands will be sent to rover\n")
        system.serial_port = None
    
    system.run()

