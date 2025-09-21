#!/usr/bin/env python3
"""
Autonomous Environmental Monitoring Platform
Integrates BME680 data logging with GPS navigation and motor control
Modified to stay at home until receiving laboratory commands
"""

import serial
import csv
import os
import time
import json
import socket
import threading
import math
from datetime import datetime
import logging
import sys
import glob
import subprocess
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

class AutonomousPlatform:
    def __init__(self, config_file='platform_config.json'):
        """Initialize the autonomous platform system"""
        
        # Setup logging FIRST - before anything else that might use self.logger
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('platform.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        self.load_config(config_file)
        
        # Initialize components
        self.bme680_serial = None
        self.gps_serial = None
        self.lab_socket = None
        
        # GPS and navigation
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.gps_fix = False
        
        self.current_speed_limit = self.config['max_speed']  # Current speed limit

        # Mission control - MODIFIED: Start with mission inactive
        self.mission_active = False
        self.returning_home = False
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.at_home = True  # NEW: Track if platform is at home
        self.home_tolerance = self.config.get('home_tolerance', 2.0)  # NEW: Tolerance for being "at home"
        
        # Motor control setup
        self.setup_motor_control()
        
        # Data storage
        self.data_buffer = []
        self.log_file_path = None
        
        self.lab_connection_active = False
        self.last_lab_connection_attempt = 0
        self.lab_connection_retry_interval = 10  # seconds - try every 10 seconds
        
        self.pending_file_transfers = []
        self.mission_start_time = None
        
        self.mission_waypoints = []  # List of waypoint coordinates
        self.current_waypoint_index = 0
        self.waypoints_completed = []  # Track completed waypoints
        
        self.last_transfer_attempt = 0
        self.transfer_cooldown = 30  
        self.in_transfer_range = False
        self.last_range_check = 0

    def load_config(self, config_file):
        """Load configuration from JSON file"""
        default_config = {
            "bme680_device": "/dev/ttyUSB0",
            "bme680_baud": 115200,
            "gps_device": "/dev/ttyAMA0",
            "gps_baud": 9600,
            "usb_mount_point": "/media/user/SDCARD",
            "lab_server_ip": "your_ip",
            "lab_server_port": 8080,
            "steering_gpio": 18,
            "throttle_gpio": 19,
            "max_speed": 0.3,
            "navigation_tolerance": 5.0,
            "data_collection_interval": 5,
            "home_coordinates": {"lat": 0.0, "lon": 0.0},
            "transfer_range_meters": 10.0,  
            "transfer_check_interval": 30,
            "min_speed": 0.0,           # Minimum speed limit
            "speed_step": 0.05,         # Speed adjustment step
            "home_tolerance": 2.0       # NEW: Distance tolerance for being "at home" (meters)
        }
        
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    # Merge with defaults
                    for key, value in default_config.items():
                        if key not in config:
                            config[key] = value
            else:
                config = default_config
                # Save default config
                with open(config_file, 'w') as f:
                    json.dump(config, f, indent=4)
                    
            self.config = config
            self.logger.info(f"Configuration loaded from {config_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            self.config = default_config

    def check_at_home(self):
        """Check if platform is currently at home position"""
        if not self.gps_fix:
            return False
            
        home_lat = self.config['home_coordinates']['lat']
        home_lon = self.config['home_coordinates']['lon']
        
        if home_lat == 0.0 and home_lon == 0.0:
            return False  # Home not set
        
        distance_to_home = self.calculate_distance(
            self.current_lat, self.current_lon,
            home_lat, home_lon
        )
        
        was_at_home = self.at_home
        self.at_home = distance_to_home <= self.home_tolerance
        
        # Log status changes
        if was_at_home != self.at_home:
            if self.at_home:
                self.logger.info(f"Platform arrived at home (distance: {distance_to_home:.2f}m)")
            else:
                self.logger.info(f"Platform left home (distance: {distance_to_home:.2f}m)")
        
        return self.at_home

    def navigate_to_target(self):
        """Enhanced navigation with home position awareness"""
        if not self.gps_fix:
            if hasattr(self, '_last_no_gps_warning'):
                if time.time() - self._last_no_gps_warning > 60:  # Warn every minute
                    self.logger.warning("Navigation disabled: No GPS fix")
                    self._last_no_gps_warning = time.time()
            else:
                self.logger.warning("Navigation disabled: No GPS fix")
                self._last_no_gps_warning = time.time()
            self.stop_motors()
            return
        
        # Update home status
        self.check_at_home()
        
        try:
            self.check_transfer_range()
        except Exception as e:
            self.logger.debug(f"Range check during navigation: {e}")
        
        # MODIFIED: Only navigate if mission is active OR returning home
        if not self.mission_active and not self.returning_home:
            self.stop_motors()
            # If we're not at home and no mission is active, we should return home
            if not self.at_home and self.config['home_coordinates']['lat'] != 0.0:
                self.logger.info("Not at home and no active mission - returning home")
                self.return_home()
            return
        
        try:
            # Calculate distance to current target
            distance = self.calculate_distance(
                self.current_lat, self.current_lon,
                self.target_lat, self.target_lon
            )
            
            # Check if we've reached the current waypoint
            if distance < self.config['navigation_tolerance']:
                current_waypoint = {
                    'lat': self.target_lat, 
                    'lon': self.target_lon,
                    'reached_time': datetime.now().isoformat(),
                    'waypoint_index': self.current_waypoint_index
                }
                self.waypoints_completed.append(current_waypoint)
                
                self.logger.info(f"Waypoint {self.current_waypoint_index + 1} reached! Distance: {distance:.2f}m")
                
                # Check if there are more waypoints
                if self.current_waypoint_index + 1 < len(self.mission_waypoints):
                    # Move to next waypoint
                    self.current_waypoint_index += 1
                    next_waypoint = self.mission_waypoints[self.current_waypoint_index]
                    self.target_lat = next_waypoint['lat']
                    self.target_lon = next_waypoint['lon']
                    
                    remaining_waypoints = len(self.mission_waypoints) - self.current_waypoint_index
                    self.logger.info(f"Moving to waypoint {self.current_waypoint_index + 1}: {self.target_lat}, {self.target_lon}")
                    self.logger.info(f"Waypoints remaining: {remaining_waypoints}")
                    
                    # Send waypoint completion update to lab
                    self.send_waypoint_update(current_waypoint, 'COMPLETED')
                    
                else:
                    # All waypoints completed
                    self.logger.info("All waypoints completed!")
                    self.stop_motors()
                    self.mission_active = False
                    
                    # Send mission completion update
                    self.send_mission_completion_update()
                    
                    # If we were returning home, we're now home
                    if self.returning_home:
                        self.returning_home = False
                        self.logger.info("Successfully returned home - platform ready for new commands")
                    else:
                        # Mission completed, automatically return home
                        self.logger.info("Mission completed - automatically returning home")
                        self.return_home()
                    
                    # Transfer files from completed mission
                    self.transfer_mission_files()
                    
                    return
            
            # Continue navigation to current target
            # Calculate bearing to target
            bearing = self.calculate_bearing(
                self.current_lat, self.current_lon,
                self.target_lat, self.target_lon
            )
            
            # Simplified navigation (you'll need compass/IMU for proper steering)
            steering_command = 0  # Placeholder - needs compass integration
            
            # Set throttle based on distance (closer = slower)
            max_distance = 100  # meters
            min_throttle = 0.1   # Minimum throttle to overcome friction
            
            throttle_ratio = min(distance / max_distance, 1.0)
            throttle_command = min_throttle + (throttle_ratio * (self.current_speed_limit - min_throttle))
            
            # Apply motor commands with bounds checking
            if self.steering_servo and self.throttle_servo:
                steering_value = max(-1, min(1, steering_command))
                throttle_value = max(0, min(1, throttle_command))
                
                self.steering_servo.value = steering_value
                self.throttle_servo.value = throttle_value
                
                # Log navigation info occasionally
                if hasattr(self, '_last_nav_log_time'):
                    if time.time() - self._last_nav_log_time > 10:  # Every 10 seconds
                        if self.returning_home:
                            nav_info = "Returning Home"
                        else:
                            nav_info = f"WP{self.current_waypoint_index + 1}/{len(self.mission_waypoints)}"
                        self.logger.info(f"Navigating {nav_info}: Distance={distance:.1f}m, Bearing={bearing:.0f}°, Throttle={throttle_value:.2f}")
                        self._last_nav_log_time = time.time()
                else:
                    if self.returning_home:
                        nav_info = "Returning Home"
                    else:
                        nav_info = f"WP{self.current_waypoint_index + 1}/{len(self.mission_waypoints)}"
                    self.logger.info(f"Navigation started {nav_info}: Distance={distance:.1f}m, Bearing={bearing:.0f}°")
                    self._last_nav_log_time = time.time()
            else:
                self.logger.error("Motor servos not available for navigation")
                self.mission_active = False
                self.returning_home = False
                
        except Exception as e:
            self.logger.error(f"Navigation error: {e}")
            self.stop_motors()
            self.mission_active = False
            self.returning_home = False

    def return_home(self):
        """Return to home coordinates"""
        home_lat = self.config['home_coordinates']['lat']
        home_lon = self.config['home_coordinates']['lon']
        
        if home_lat == 0.0 and home_lon == 0.0:
            self.logger.error("Cannot return home: Home coordinates not set")
            return False
        
        self.logger.info(f"Returning home to coordinates: {home_lat}, {home_lon}")
        self.target_lat = home_lat
        self.target_lon = home_lon
        self.returning_home = True
        self.mission_active = False  # Clear any active mission
        
        # Create a single waypoint mission for returning home
        self.mission_waypoints = [{'lat': home_lat, 'lon': home_lon}]
        self.current_waypoint_index = 0
        self.waypoints_completed = []
        
        return True

    def handle_lab_command(self, command):
        """Handle specific laboratory commands with home position awareness"""
        cmd_type = command.get('type')
        
        if cmd_type == 'GOTO':
            # Check if we're at home before starting mission
            if not self.at_home and self.config['home_coordinates']['lat'] != 0.0:
                self.logger.warning("Platform not at home - must return home before starting new mission")
                error_response = {
                    'type': 'MISSION_REJECTED',
                    'reason': 'Platform not at home position',
                    'current_position': {'lat': self.current_lat, 'lon': self.current_lon},
                    'home_position': self.config['home_coordinates'],
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_response)
                return
            
            # Mark mission start time
            self.mission_start_time = datetime.now()
            self.target_lat = command['lat']
            self.target_lon = command['lon']
            self.mission_waypoints = [{'lat': command['lat'], 'lon': command['lon']}]
            self.current_waypoint_index = 0
            self.waypoints_completed = []
            self.mission_active = True
            self.returning_home = False
            self.logger.info(f"New mission accepted: Go to {self.target_lat}, {self.target_lon}")
            
            # Send mission acceptance confirmation
            confirmation = {
                'type': 'MISSION_ACCEPTED',
                'mission_type': 'SINGLE_WAYPOINT',
                'target': {'lat': self.target_lat, 'lon': self.target_lon},
                'timestamp': datetime.now().isoformat()
            }
            self.send_data_to_lab(confirmation)

        elif cmd_type == 'MISSION_PATH':
            # Check if we're at home before starting mission
            if not self.at_home and self.config['home_coordinates']['lat'] != 0.0:
                self.logger.warning("Platform not at home - must return home before starting new mission")
                error_response = {
                    'type': 'MISSION_REJECTED',
                    'reason': 'Platform not at home position',
                    'current_position': {'lat': self.current_lat, 'lon': self.current_lon},
                    'home_position': self.config['home_coordinates'],
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_response)
                return
            
            # Handle multiple waypoints
            self.mission_start_time = datetime.now()
            self.mission_waypoints = command['waypoints']  # List of {'lat': x, 'lon': y} objects
            self.current_waypoint_index = 0
            self.waypoints_completed = []
            
            if self.mission_waypoints:
                # Set first waypoint as current target
                first_waypoint = self.mission_waypoints[0]
                self.target_lat = first_waypoint['lat']
                self.target_lon = first_waypoint['lon']
                self.mission_active = True
                self.returning_home = False
                
                waypoint_summary = [(wp['lat'], wp['lon']) for wp in self.mission_waypoints]
                self.logger.info(f"Mission path accepted: {len(self.mission_waypoints)} waypoints")
                self.logger.info(f"Waypoints: {waypoint_summary}")
                self.logger.info(f"Starting with waypoint 1: {self.target_lat}, {self.target_lon}")
                
                # Send mission acceptance confirmation
                confirmation = {
                    'type': 'MISSION_ACCEPTED',
                    'mission_type': 'MULTI_WAYPOINT',
                    'waypoint_count': len(self.mission_waypoints),
                    'waypoints': self.mission_waypoints,
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(confirmation)
            else:
                self.logger.error("Empty waypoint list received")
                error_response = {
                    'type': 'MISSION_REJECTED',
                    'reason': 'Empty waypoint list',
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_response)
            
        elif cmd_type == 'RETURN_HOME':
            if self.return_home():
                self.logger.info("Return home command accepted")
                confirmation = {
                    'type': 'RETURN_HOME_ACCEPTED',
                    'home_coordinates': self.config['home_coordinates'],
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(confirmation)
            else:
                error_response = {
                    'type': 'RETURN_HOME_REJECTED',
                    'reason': 'Home coordinates not set',
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_response)
            
        elif cmd_type == 'EMERGENCY_STOP':
            self.emergency_stop()
            
        elif cmd_type == 'SET_HOME':
            # Only allow setting home if platform is stationary
            if not self.mission_active and not self.returning_home:
                self.config['home_coordinates']['lat'] = command['lat']
                self.config['home_coordinates']['lon'] = command['lon']
                self.logger.info(f"Home set to {command['lat']}, {command['lon']}")
                
                # Update our at_home status
                self.check_at_home()
                
                confirmation = {
                    'type': 'HOME_SET_CONFIRM',
                    'coordinates': self.config['home_coordinates'],
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(confirmation)
            else:
                error_response = {
                    'type': 'HOME_SET_REJECTED',
                    'reason': 'Cannot set home while mission is active',
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_response)
            
        elif cmd_type == 'STATUS_REQUEST':
            self.send_status_update()
            
        elif cmd_type == 'SET_SPEED':
            new_speed = command.get('speed', self.config['max_speed'])
            # Validate speed limits
            min_speed = self.config.get('min_speed', 0.1)
            max_speed = self.config['max_speed']
            
            if min_speed <= new_speed <= max_speed:
                self.current_speed_limit = new_speed
                self.logger.info(f"Speed limit set to {new_speed:.2f}")
                # Send confirmation back to lab
                confirmation = {
                    'type': 'SPEED_SET_CONFIRM',
                    'speed': self.current_speed_limit,
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(confirmation)
            else:
                self.logger.warning(f"Invalid speed {new_speed}. Must be between {min_speed} and {max_speed}")
                error_msg = {
                    'type': 'SPEED_SET_ERROR',
                    'error': f'Speed must be between {min_speed} and {max_speed}',
                    'requested_speed': new_speed,
                    'current_speed': self.current_speed_limit,
                    'timestamp': datetime.now().isoformat()
                }
                self.send_data_to_lab(error_msg)
        
        elif cmd_type == 'INCREASE_SPEED':
            step = command.get('step', self.config.get('speed_step', 0.05))
            new_speed = min(self.current_speed_limit + step, self.config['max_speed'])
            if new_speed != self.current_speed_limit:
                self.current_speed_limit = new_speed
                self.logger.info(f"Speed increased to {self.current_speed_limit:.2f}")
            else:
                self.logger.info("Already at maximum speed")
            
            confirmation = {
                'type': 'SPEED_CHANGE_CONFIRM',
                'action': 'INCREASE',
                'speed': self.current_speed_limit,
                'timestamp': datetime.now().isoformat()
            }
            self.send_data_to_lab(confirmation)
        
        elif cmd_type == 'DECREASE_SPEED':
            step = command.get('step', self.config.get('speed_step', 0.05))
            min_speed = self.config.get('min_speed', 0.1)
            new_speed = max(self.current_speed_limit - step, min_speed)
            if new_speed != self.current_speed_limit:
                self.current_speed_limit = new_speed
                self.logger.info(f"Speed decreased to {self.current_speed_limit:.2f}")
            else:
                self.logger.info("Already at minimum speed")
            
            confirmation = {
                'type': 'SPEED_CHANGE_CONFIRM',
                'action': 'DECREASE',
                'speed': self.current_speed_limit,
                'timestamp': datetime.now().isoformat()
            }
            self.send_data_to_lab(confirmation)

    def send_status_update(self):
        """Send current status to laboratory with home position info"""
        # Calculate distance to home for status
        distance_to_home = None
        if (self.gps_fix and 
            self.config['home_coordinates']['lat'] != 0.0 and 
            self.config['home_coordinates']['lon'] != 0.0):
            distance_to_home = self.calculate_distance(
                self.current_lat, self.current_lon,
                self.config['home_coordinates']['lat'],
                self.config['home_coordinates']['lon']
            )
            
        status = {
            'type': 'STATUS_UPDATE',
            'timestamp': datetime.now().isoformat(),
            'position': {
                'lat': self.current_lat,
                'lon': self.current_lon,
                'gps_fix': self.gps_fix,
                'distance_to_home': distance_to_home,
                'in_transfer_range': self.in_transfer_range,
                'at_home': self.at_home  # NEW: Include home status
            },
            'mission': {
                'active': self.mission_active,
                'target_lat': self.target_lat,
                'target_lon': self.target_lon,
                'returning_home': self.returning_home,
                'waypoint_progress': {
                    'current': self.current_waypoint_index + 1 if self.mission_waypoints else 0,
                    'total': len(self.mission_waypoints),
                    'completed': len(self.waypoints_completed)
                } if self.mission_waypoints else None
            },
            'speed_control': {
                'current_limit': self.current_speed_limit,
                'max_speed': self.config['max_speed'],
                'min_speed': self.config.get('min_speed', 0.1)
            },
            'home_status': {  # NEW: Detailed home status
                'coordinates': self.config['home_coordinates'],
                'tolerance': self.home_tolerance,
                'at_home': self.at_home
            },
            'sensor_data': self.data_buffer[-1] if self.data_buffer else None
        }
        self.send_data_to_lab(status)

    def emergency_stop(self):
        """Emergency stop all operations"""
        self.logger.warning("EMERGENCY STOP ACTIVATED")
        self.stop_motors()
        self.mission_active = False
        self.returning_home = False
        
        # Send emergency stop confirmation
        confirmation = {
            'type': 'EMERGENCY_STOP_CONFIRM',
            'timestamp': datetime.now().isoformat(),
            'position': {'lat': self.current_lat, 'lon': self.current_lon}
        }
        self.send_data_to_lab(confirmation)

    def navigation_thread(self):
        """Navigation control thread with improved home management"""
        while True:
            try:
                if self.mission_active or self.returning_home:
                    self.navigate_to_target()
                else:
                    self.stop_motors()
                    
                    # Update home status when not in mission
                    self.check_at_home()
                    
                    try:
                        self.check_transfer_range()
                    except Exception as e:
                        self.logger.debug(f"Range check error: {e}")
                
                # Process any pending lab commands
                self.process_lab_commands()
                
                # Send periodic status updates (every 30 seconds)
                if not hasattr(self, '_last_status_update') or time.time() - self._last_status_update > 30:
                    self.send_status_update()
                    self._last_status_update = time.time()
                    
            except Exception as e:
                self.logger.error(f"Navigation thread error: {e}")
            time.sleep(1)

    # Include all other methods from the original class (setup_motor_control, setup_bme680_serial, etc.)
    # For brevity, I'm only showing the modified methods above
    # The rest of the methods remain the same as in the original code

    def run(self):
        """Main execution method with home position initialization"""
        self.logger.info("Starting Autonomous Environmental Monitoring Platform")
        
        try:
            # Set home coordinates to current position if not set
            if self.config['home_coordinates']['lat'] == 0.0:
                self.logger.info("Waiting for GPS fix to set home coordinates...")
                if self.setup_gps_serial():
                    # Wait for GPS fix
                    for i in range(60):  # Wait up to 60 seconds
                        if self.gps_serial and self.gps_serial.in_waiting > 0:
                            gps_line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                            if self.parse_nmea_gps(gps_line) and self.gps_fix:
                                self.config['home_coordinates']['lat'] = self.current_lat
                                self.config['home_coordinates']['lon'] = self.current_lon
                                self.home_lat = self.current_lat
                                self.home_lon = self.current_lon
                                self.at_home = True  # We're setting home to current position
                                self.logger.info(f"Home coordinates set to current position: {self.current_lat:.6f}, {self.current_lon:.6f}")
                                break
                        time.sleep(1)
                    
                    if not self.gps_fix:
                        self.logger.warning("Could not get GPS fix to set home coordinates")
                        self.logger.info("Platform will wait for laboratory commands to set home manually")
            else:
                # Home coordinates already set, check if we're at home
                self.logger.info(f"Home coordinates loaded: {self.config['home_coordinates']['lat']:.6f}, {self.config['home_coordinates']['lon']:.6f}")
            
            # Initialize connections
            if not self.setup_bme680_serial():
                self.logger.warning("BME680 not available - continuing without sensor data")
            
            if not self.setup_gps_serial():
                self.logger.warning("GPS not available - navigation disabled")
            
            if not self.setup_lab_communication():
                self.logger.warning("Lab communication not available - running standalone")
            
            # Start threads
            threads = []
            
            if self.gps_serial:
                gps_thread = threading.Thread(target=self.gps_thread, daemon=True)
                gps_thread.start()
                threads.append(gps_thread)
                self.logger.info("GPS thread started")
            
            if self.bme680_serial:
                bme_thread = threading.Thread(target=self.bme680_thread, daemon=True)
                bme_thread.start()
                threads.append(bme_thread)
                self.logger.info("BME680 thread started")
            
            nav_thread = threading.Thread(target=self.navigation_thread, daemon=True)
            nav_thread.start()
            threads.append(nav_thread)
            self.logger.info("Navigation thread started")
            
            # Start lab connection monitor thread
            lab_monitor_thread = threading.Thread(target=self.lab_connection_monitor_thread, daemon=True)
            lab_monitor_thread.start()
            threads.append(lab_monitor_thread)
            self.logger.info("Lab connection monitor thread started (10-second retry interval)")
            
            # Main loop
            self.logger.info("Platform operational and ready for laboratory commands")
            self.logger.info("Platform will stay at home until receiving mission commands")
            try:
                while True:
                    time.sleep(1)
                    # Perform any periodic main thread tasks here
                    
            except KeyboardInterrupt:
                self.logger.info("Shutdown requested by user")
                
        except Exception as e:
            self.logger.error(f"Platform error: {e}")
            
        finally:
            self.cleanup()

    def setup_motor_control(self):
        """Initialize PWM control for steering and throttle"""
        try:
            # Use pigpio for more precise PWM control
            factory = PiGPIOFactory()
            
            # Initialize steering servo (typical servo range: -1 to +1)
            self.steering_servo = Servo(
                self.config['steering_gpio'], 
                pin_factory=factory,
                min_pulse_width=1.0/1000,  # 1ms
                max_pulse_width=2.0/1000   # 2ms
            )
            
            # Initialize throttle control (ESC range: -1 to +1)
            self.throttle_servo = Servo(
                self.config['throttle_gpio'], 
                pin_factory=factory,
                min_pulse_width=1.0/1000,  # 1ms
                max_pulse_width=2.0/1000   # 2ms
            )
            
            # Initialize to neutral positions
            self.steering_servo.value = 0  # Straight
            self.throttle_servo.value = 0  # Stop
            
            self.logger.info("Motor control initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize motor control: {e}")
            self.steering_servo = None
            self.throttle_servo = None

    def test_motors_only(self):
        """Simple motor test without full system initialization"""
        self.logger.info("Starting motor test...")
        
        if not self.steering_servo or not self.throttle_servo:
            self.logger.error("Motors not initialized properly")
            return False
        
        try:
            self.logger.info("Testing forward motion...")
            self.throttle_servo.value = 0.5  # Slow forward
            time.sleep(2)
            
            self.logger.info("Testing steering...")
            self.steering_servo.value = 0.5   # Turn right
            time.sleep(2)
            
            self.steering_servo.value = -0.5  # Turn left
            time.sleep(2)
            
            self.logger.info("Stopping motors...")
            self.stop_motors()
            
            self.logger.info("Motor test completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Motor test failed: {e}")
            self.stop_motors()
            return False

    def setup_bme680_serial(self):
        """Initialize BME680 serial connection"""
        try:
            self.bme680_serial = serial.Serial(
                port=self.config['bme680_device'],
                baudrate=self.config['bme680_baud'],
                timeout=1
            )
            self.logger.info("BME680 serial connection established")
            return True
        except Exception as e:
            self.logger.error(f"Failed to setup BME680 serial: {e}")
            return False

    def setup_gps_serial(self):
        """Initialize GPS serial connection with enhanced diagnostics"""
        try:
            # Check if the port exists
            if not os.path.exists(self.config['gps_device']):
                self.logger.error(f"GPS device {self.config['gps_device']} does not exist")
                self.logger.info("Make sure UART is enabled in /boot/config.txt with 'enable_uart=1'")
                return False
            
            # Check permissions
            if not os.access(self.config['gps_device'], os.R_OK | os.W_OK):
                self.logger.error(f"No permission to access {self.config['gps_device']}")
                self.logger.info("Add user to dialout group: sudo usermod -a -G dialout $USER")
                return False
            
            self.gps_serial = serial.Serial(
                port=self.config['gps_device'],
                baudrate=self.config['gps_baud'],
                timeout=2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            # Clear any existing data in buffer
            self.gps_serial.flushInput()
            
            # Enhanced GPS testing with detailed diagnostics
            self.logger.info("Testing GPS data stream with detailed analysis...")
            
            sentence_types = {}
            data_received = False
            fix_quality_found = False
            
            for attempt in range(30):  # Extended to 30 seconds
                try:
                    if self.gps_serial.in_waiting > 0:
                        line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                        
                        if line and line.startswith('):
                            # Track sentence types
                            sentence_type = line.split(',')[0]
                            sentence_types[sentence_type] = sentence_types.get(sentence_type, 0) + 1
                            
                            data_received = True
                            
                            # Log first few complete sentences for debugging
                            if attempt < 5:
                                self.logger.info(f"GPS NMEA: {line}")
                            
                            # Check for fix quality in GGA sentences
                            if 'GGA' in sentence_type:
                                parts = line.split(',')
                                if len(parts) > 6 and parts[6]:
                                    quality = parts[6]
                                    if quality != '0':
                                        fix_quality_found = True
                                        self.logger.info(f"GPS fix quality detected: {quality}")
                                    else:
                                        self.logger.debug(f"GPS no fix yet (quality=0) - attempt {attempt}")
                            
                            # Check RMC status
                            elif 'RMC' in sentence_type:
                                parts = line.split(',')
                                if len(parts) > 2:
                                    status = parts[2]
                                    if status == 'A':
                                        self.logger.info("GPS RMC status: Active (valid fix)")
                                    else:
                                        self.logger.debug(f"GPS RMC status: {status} (no fix yet)")
                            
                            # Break early if we get a valid fix
                            if fix_quality_found:
                                break
                                
                except Exception as e:
                    self.logger.debug(f"GPS read attempt {attempt}: {e}")
                time.sleep(1)
            
            # Report findings
            self.logger.info(f"GPS initialization complete. Sentences detected: {sentence_types}")
            
            if not data_received:
                self.logger.error("No GPS NMEA data received")
                self.logger.info("Troubleshooting steps:")
                self.logger.info("1. Check GPS module power (usually 3.3V or 5V)")
                self.logger.info("2. Verify RX/TX connections")
                self.logger.info("3. Ensure GPS antenna is connected")
                return False
            
            if not fix_quality_found:
                self.logger.warning("GPS communication OK but no position fix detected")
                self.logger.info("This is normal for:")
                self.logger.info("- First GPS startup (cold start can take 30+ minutes)")
                self.logger.info("- Indoor or obstructed locations")
                self.logger.info("- Poor weather conditions")
                self.logger.info("GPS will continue trying to acquire fix...")
            
            self.logger.info("GPS serial connection established")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"Serial connection failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Failed to setup GPS serial: {e}")
            return False

    def parse_nmea_gps(self, nmea_sentence):
        """Enhanced NMEA GPS sentence parser with detailed logging"""
        try:
            if not nmea_sentence or len(nmea_sentence) < 10:
                return False
            
            # Handle different NMEA sentence types
            if nmea_sentence.startswith('$GPGGA') or nmea_sentence.startswith('$GNGGA'):
                # GGA - Global Positioning System Fix Data
                parts = nmea_sentence.split(',')
                if len(parts) < 15:
                    self.logger.debug(f"Incomplete GGA sentence: {len(parts)} parts")
                    return False
                
                # Log detailed GGA analysis occasionally
                if not hasattr(self, '_last_gga_analysis') or time.time() - self._last_gga_analysis > 60:
                    self.logger.info(f"GGA Analysis - Quality:{parts[6]}, Satellites:{parts[7]}, HDOP:{parts[8]}")
                    self._last_gga_analysis = time.time()
                
                # Check if we have valid data (quality indicator)
                if not parts[6] or parts[6] == '0':
                    self.gps_fix = False
                    # Log why no fix occasionally
                    if not hasattr(self, '_last_no_fix_log') or time.time() - self._last_no_fix_log > 300:
                        self.logger.info(f"No GPS fix: Quality={parts[6]}, Satellites={parts[7]}")
                        self._last_no_fix_log = time.time()
                    return False
                
                if parts[2] and parts[4]:  # Lat and Lon present
                    try:
                        # Parse latitude
                        lat_raw = float(parts[2])
                        lat_deg = int(lat_raw / 100)
                        lat_min = lat_raw - (lat_deg * 100)
                        lat = lat_deg + (lat_min / 60.0)
                        if parts[3] == 'S':
                            lat = -lat
                        
                        # Parse longitude
                        lon_raw = float(parts[4])
                        lon_deg = int(lon_raw / 100)
                        lon_min = lon_raw - (lon_deg * 100)
                        lon = lon_deg + (lon_min / 60.0)
                        if parts[5] == 'W':
                            lon = -lon
                        
                        # Validate coordinates (basic sanity check)
                        if -90 <= lat <= 90 and -180 <= lon <= 180:
                            # Check if this is a significant position change
                            if abs(lat - self.current_lat) > 0.0001 or abs(lon - self.current_lon) > 0.0001:
                                self.logger.info(f"GPS position update: {lat:.6f}, {lon:.6f}")
                            
                            self.current_lat = lat
                            self.current_lon = lon
                            self.gps_fix = True
                            
                            # Log fix details occasionally
                            if not hasattr(self, '_last_gps_detail_log') or time.time() - self._last_gps_detail_log > 60:
                                self.logger.info(f"GPS Details - Lat:{lat:.6f}, Lon:{lon:.6f}, Quality:{parts[6]}, Sats:{parts[7]}")
                                self._last_gps_detail_log = time.time()
                            
                            return True
                        else:
                            self.logger.warning(f"Invalid GPS coordinates: {lat}, {lon}")
                            
                    except ValueError as e:
                        self.logger.debug(f"GPS coordinate parsing error: {e}")
                        
            elif nmea_sentence.startswith('$GPRMC') or nmea_sentence.startswith('$GNRMC'):
                # RMC - Recommended Minimum Navigation Information
                parts = nmea_sentence.split(',')
                if len(parts) < 12:
                    return False
                
                # Log RMC status occasionally for debugging
                if not hasattr(self, '_last_rmc_log') or time.time() - self._last_rmc_log > 120:
                    self.logger.debug(f"RMC Status: {parts[2]} ({'Valid' if parts[2] == 'A' else 'Invalid'})")
                    self._last_rmc_log = time.time()
                
                # Check if data is valid
                if parts[2] != 'A':  # 'A' = Active, 'V' = Void
                    self.gps_fix = False
                    return False
                    
                if parts[3] and parts[5]:  # Lat and Lon present
                    try:
                        # Parse latitude
                        lat_raw = float(parts[3])
                        lat_deg = int(lat_raw / 100)
                        lat_min = lat_raw - (lat_deg * 100)
                        lat = lat_deg + (lat_min / 60.0)
                        if parts[4] == 'S':
                            lat = -lat
                        
                        # Parse longitude
                        lon_raw = float(parts[5])
                        lon_deg = int(lon_raw / 100)
                        lon_min = lon_raw - (lon_deg * 100)
                        lon = lon_deg + (lon_min / 60.0)
                        if parts[6] == 'W':
                            lon = -lon
                        
                        # Validate coordinates
                        if -90 <= lat <= 90 and -180 <= lon <= 180:
                            self.current_lat = lat
                            self.current_lon = lon
                            self.gps_fix = True
                            return True
                            
                    except ValueError as e:
                        self.logger.debug(f"RMC coordinate parsing error: {e}")
                        
        except Exception as e:
            self.logger.debug(f"NMEA parsing error: {e}")
        
        return False

    def check_gps_health(self):
        """Perform GPS health check and provide recommendations"""
        self.logger.info("Performing GPS health check...")
        
        if not self.gps_serial:
            self.logger.error("GPS serial connection not established")
            return False
        
        # Collect data for 30 seconds
        start_time = time.time()
        sentence_count = 0
        fix_attempts = 0
        valid_fixes = 0
        max_satellites = 0
        
        while time.time() - start_time < 30:
            try:
                if self.gps_serial.in_waiting > 0:
                    line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('):
                        sentence_count += 1
                        
                        if 'GGA' in line:
                            fix_attempts += 1
                            parts = line.split(',')
                            if len(parts) > 6 and parts[6] and parts[6] != '0':
                                valid_fixes += 1
                            
                            # Check satellite count
                            if len(parts) > 7 and parts[7]:
                                try:
                                    sat_count = int(parts[7])
                                    max_satellites = max(max_satellites, sat_count)
                                except:
                                    pass
            except:
                pass
            time.sleep(0.1)
        
        # Report health status
        self.logger.info("=== GPS Health Report ===")
        self.logger.info(f"NMEA sentences in 30s: {sentence_count}")
        self.logger.info(f"Fix attempts: {fix_attempts}")
        self.logger.info(f"Valid fixes: {valid_fixes}")
        self.logger.info(f"Maximum satellites: {max_satellites}")
        
        if sentence_count == 0:
            self.logger.error("No GPS data - check wiring and power")
            return False
        elif max_satellites < 4:
            self.logger.warning("Insufficient satellites - move to open area")
            return False
        elif valid_fixes == 0:
            self.logger.warning("No valid fixes - GPS needs more time or better location")
            return False
        else:
            self.logger.info("GPS health check passed")
            return True

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates (meters)"""
        R = 6371000  # Earth's radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from current position to target (degrees)"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.atan2(y, x)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360  # Normalize to 0-360
        
        return bearing

    def send_waypoint_update(self, waypoint_data, status):
        """Send waypoint completion update to laboratory"""
        update = {
            'type': 'WAYPOINT_UPDATE',
            'status': status,  # 'COMPLETED', 'SKIPPED', etc.
            'waypoint': waypoint_data,
            'mission_progress': {
                'completed': len(self.waypoints_completed),
                'total': len(self.mission_waypoints),
                'current_index': self.current_waypoint_index
            },
            'timestamp': datetime.now().isoformat()
        }
        self.send_data_to_lab(update)

    def send_mission_completion_update(self):
        """Send mission completion summary to laboratory"""
        completion_update = {
            'type': 'MISSION_COMPLETE',
            'mission_summary': {
                'start_time': self.mission_start_time.isoformat() if self.mission_start_time else None,
                'end_time': datetime.now().isoformat(),
                'total_waypoints': len(self.mission_waypoints),
                'completed_waypoints': len(self.waypoints_completed),
                'waypoints_completed': self.waypoints_completed
            },
            'timestamp': datetime.now().isoformat()
        }
        self.send_data_to_lab(completion_update)

    def stop_motors(self):
        """Stop all motors"""
        if self.steering_servo and self.throttle_servo:
            self.steering_servo.value = 0
            self.throttle_servo.value = 0

    def setup_lab_communication(self):
        """Setup TCP connection to laboratory with timeout and enhanced logging"""
        try:
            # Log connection attempt
            self.logger.info(f"Attempting connection to lab server {self.config['lab_server_ip']}:{self.config['lab_server_port']}")
            
            self.lab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.lab_socket.settimeout(5)  # Reduced timeout for faster retry cycles
            self.lab_socket.connect((
                self.config['lab_server_ip'], 
                self.config['lab_server_port']
            ))
            self.lab_socket.settimeout(None)  # Remove timeout for normal operations
            self.lab_connection_active = True
            self.logger.info("✓ Successfully connected to laboratory server")
            
            # Send initial connection status
            connection_status = {
                'type': 'PLATFORM_CONNECTED',
                'platform_id': 'environmental_platform_01',  # You can make this configurable
                'timestamp': datetime.now().isoformat(),
                'status': 'online'
            }
            self.send_data_to_lab(connection_status)
            
            return True
            
        except socket.timeout:
            self.logger.warning(f"✗ Lab connection timed out (server: {self.config['lab_server_ip']}:{self.config['lab_server_port']})")
            self.lab_connection_active = False
            return False
        except ConnectionRefusedError:
            self.logger.warning(f"✗ Lab connection refused (server may be down: {self.config['lab_server_ip']}:{self.config['lab_server_port']})")
            self.lab_connection_active = False
            return False
        except Exception as e:
            self.logger.warning(f"✗ Failed to connect to lab: {e}")
            self.lab_connection_active = False
            return False

    def send_data_to_lab(self, data):
        """Send data to laboratory with frequent connection retry"""
        try:
            if self.lab_socket and self.lab_connection_active:
                message = json.dumps(data) + '\n'
                self.lab_socket.send(message.encode())
                return True
            else:
                # Try to reconnect frequently (every 10 seconds)
                current_time = time.time()
                if current_time - self.last_lab_connection_attempt >= self.lab_connection_retry_interval:
                    self.last_lab_connection_attempt = current_time
                    self.logger.debug("Attempting lab reconnection...")
                    self.setup_lab_communication()
                return False
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError) as e:
            self.logger.warning(f"Lab connection lost: {e}")
            self.lab_connection_active = False
            self._close_lab_socket()
            return False
        except Exception as e:
            self.logger.error(f"Failed to send data to lab: {e}")
            self.lab_connection_active = False
            self._close_lab_socket()
            return False

    def _close_lab_socket(self):
        """Helper method to safely close lab socket"""
        try:
            if self.lab_socket:
                self.lab_socket.close()
                self.lab_socket = None
        except:
            pass

    def process_lab_commands(self):
        """Process commands from laboratory with enhanced connection handling"""
        try:
            if self.lab_socket and self.lab_connection_active:
                # Set socket to non-blocking to avoid hanging
                self.lab_socket.settimeout(0.1)
                data = self.lab_socket.recv(1024)
                if data:
                    command = json.loads(data.decode())
                    self.handle_lab_command(command)
        except socket.timeout:
            # Normal timeout - no data available
            pass
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError) as e:
            self.logger.warning(f"Lab connection lost during command processing: {e}")
            self.lab_connection_active = False
            self._close_lab_socket()
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON received from lab: {e}")
        except Exception as e:
            self.logger.debug(f"Lab command error: {e}")
            self.lab_connection_active = False
            self._close_lab_socket()

    def lab_connection_monitor_thread(self):
        """Dedicated thread for monitoring and maintaining lab connection"""
        while True:
            try:
                current_time = time.time()
                
                # Check if we need to attempt reconnection
                if (not self.lab_connection_active and 
                    current_time - self.last_lab_connection_attempt >= self.lab_connection_retry_interval):
                    
                    self.last_lab_connection_attempt = current_time
                    self.logger.debug("Connection monitor: Attempting lab reconnection...")
                    
                    if self.setup_lab_communication():
                        self.logger.info("Connection monitor: Lab connection restored")
                    
                # If connected, send periodic heartbeat
                elif self.lab_connection_active:
                    # Send heartbeat every 30 seconds to keep connection alive
                    if not hasattr(self, '_last_heartbeat') or current_time - self._last_heartbeat > 30:
                        heartbeat = {
                            'type': 'HEARTBEAT',
                            'timestamp': datetime.now().isoformat(),
                            'status': 'alive'
                        }
                        if self.send_data_to_lab(heartbeat):
                            self._last_heartbeat = current_time
                        else:
                            self.logger.warning("Heartbeat failed - connection may be lost")
                
            except Exception as e:
                self.logger.error(f"Lab connection monitor error: {e}")
            
            time.sleep(5)  # Check every 5 seconds

    def check_transfer_range(self):
        """Check if platform is within transfer range of home and attempt transfer"""
        try:
            if not self.gps_fix:
                return False
            
            current_time = time.time()
            
            # Only check range periodically
            if current_time - self.last_range_check < self.config['transfer_check_interval']:
                return False
            
            self.last_range_check = current_time
            
            # Calculate distance to home
            home_lat = self.config['home_coordinates']['lat']
            home_lon = self.config['home_coordinates']['lon']
            
            if home_lat == 0.0 and home_lon == 0.0:
                return False  # Home not set
            
            distance_to_home = self.calculate_distance(
                self.current_lat, self.current_lon,
                home_lat, home_lon
            )
            
            # Check if within transfer range
            within_range = distance_to_home <= self.config['transfer_range_meters']
            
            # Log range status changes
            if within_range != self.in_transfer_range:
                if within_range:
                    self.logger.info(f"Entered transfer range: {distance_to_home:.1f}m from home")
                else:
                    self.logger.info(f"Left transfer range: {distance_to_home:.1f}m from home")
            
            self.in_transfer_range = within_range
            
            # Attempt transfer if in range and enough time has passed
            if (within_range and 
                current_time - self.last_transfer_attempt >= self.transfer_cooldown):
                
                self.logger.info("Attempting range-based data transfer...")
                self.last_transfer_attempt = current_time
                
                # Find and transfer any pending data files
                self.transfer_available_files()
                return True
                
            return False
            
        except Exception as e:
            self.logger.error(f"Transfer range check error: {e}")
            return False

    def transfer_available_files(self):
        """Transfer any available data files to lab"""
        try:
            usb_path = self.config['usb_mount_point']
            if not os.path.exists(usb_path):
                self.logger.warning(f"USB mount point not accessible: {usb_path}")
                return False
            
            # Find all CSV files
            csv_files = glob.glob(os.path.join(usb_path, "BME680_data_*.csv"))
            
            if not csv_files:
                self.logger.info("No data files available for transfer")
                return True
            
            self.logger.info(f"Found {len(csv_files)} data files for transfer")
            
            successful_transfers = 0
            for file_path in csv_files:
                if self.send_file_to_lab(file_path):
                    successful_transfers += 1
                else:
                    # Don't continue if transfer fails (might be connection issue)
                    break
            
            if successful_transfers > 0:
                self.logger.info(f"Range-based transfer complete: {successful_transfers} files")
            
            return successful_transfers > 0
            
        except Exception as e:
            self.logger.error(f"Available files transfer error: {e}")
            return False

    def transfer_mission_files(self):
        """Transfer all data files from the completed mission"""
        try:
            if not self.mission_start_time:
                self.logger.warning("No mission start time recorded")
                return
            
            # Find CSV files created during this mission
            usb_path = self.config['usb_mount_point']
            if not os.path.exists(usb_path):
                self.logger.error(f"USB mount point not accessible: {usb_path}")
                return
            
            csv_files = glob.glob(os.path.join(usb_path, "BME680_data_*.csv"))
            files_to_transfer = []
            
            for file_path in csv_files:
                try:
                    # Check if file was modified during mission
                    file_mtime = datetime.fromtimestamp(os.path.getmtime(file_path))
                    if file_mtime >= self.mission_start_time:
                        files_to_transfer.append(file_path)
                except Exception as e:
                    self.logger.debug(f"Error checking file {file_path}: {e}")
            
            if not files_to_transfer:
                self.logger.info("No data files to transfer from mission")
                return
            
            self.logger.info(f"Transferring {len(files_to_transfer)} mission data files")
            
            successful_transfers = 0
            for file_path in files_to_transfer:
                if self.send_file_to_lab(file_path):
                    successful_transfers += 1
                else:
                    self.logger.error(f"Failed to transfer: {os.path.basename(file_path)}")
            
            self.logger.info(f"File transfer complete: {successful_transfers}/{len(files_to_transfer)} successful")
            
            # Reset mission start time
            self.mission_start_time = None
            
        except Exception as e:
            self.logger.error(f"Mission file transfer error: {e}")

    def send_file_to_lab(self, file_path):
        """Send file to laboratory via TCP socket"""
        try:
            if not self.lab_socket or not self.lab_connection_active:
                self.logger.error("No lab connection available for file transfer")
                return False
            
            if not os.path.exists(file_path):
                self.logger.error(f"File not found: {file_path}")
                return False
            
            file_size = os.path.getsize(file_path)
            filename = os.path.basename(file_path)
            
            # Send file transfer header
            header = {
                'type': 'FILE_TRANSFER',
                'filename': filename,
                'size': file_size,
                'timestamp': datetime.now().isoformat()
            }
            
            header_json = json.dumps(header) + '\n'
            self.lab_socket.send(header_json.encode())
            
            # Wait for acknowledgment
            self.lab_socket.settimeout(10)
            response = self.lab_socket.recv(1024).decode().strip()
            if response != 'FILE_TRANSFER_ACK':
                self.logger.error(f"Lab did not acknowledge file transfer: {response}")
                return False
            
            # Send file data in chunks
            with open(file_path, 'rb') as f:
                bytes_sent = 0
                chunk_size = 8192
                
                while bytes_sent < file_size:
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    
                    self.lab_socket.send(chunk)
                    bytes_sent += len(chunk)
                    
                    # Log progress for large files
                    if file_size > 100000 and bytes_sent % 50000 == 0:
                        progress = (bytes_sent / file_size) * 100
                        self.logger.info(f"File transfer progress: {progress:.1f}%")
            
            # Wait for completion confirmation
            response = self.lab_socket.recv(1024).decode().strip()
            if response == 'FILE_TRANSFER_COMPLETE':
                self.logger.info(f"File transfer successful: {filename} ({file_size} bytes)")
                
                # Delete local file after successful transfer
                try:
                    os.remove(file_path)
                    self.logger.info(f"Local file deleted: {filename}")
                except Exception as e:
                    self.logger.warning(f"Failed to delete local file {filename}: {e}")
                
                return True
            else:
                self.logger.error(f"File transfer failed: {response}")
                return False
                
        except Exception as e:
            self.logger.error(f"File transfer error: {e}")
            return False
        finally:
            if self.lab_socket:
                self.lab_socket.settimeout(None)

    def bme680_thread(self):
        """BME680 data collection thread"""
        while True:
            try:
                if self.bme680_serial and self.bme680_serial.in_waiting > 0:
                    data_line = self.bme680_serial.readline().decode('utf-8').strip()
                    if data_line and ',' in data_line:
                        # Parse BME680 data
                        parts = data_line.split(',')
                        if len(parts) == 5:
                            sensor_data = {
                                'timestamp': datetime.now().isoformat(),
                                'gps_lat': self.current_lat,
                                'gps_lon': self.current_lon,
                                'esp32_timestamp': int(parts[0]),
                                'temperature': float(parts[1]),
                                'humidity': float(parts[2]),
                                'pressure': float(parts[3]),
                                'gas_resistance': float(parts[4])
                            }
                            
                            # Add to buffer
                            self.data_buffer.append(sensor_data)
                            if len(self.data_buffer) > 100:
                                self.data_buffer.pop(0)
                            
                            # Save to local file
                            self.save_sensor_data(sensor_data)
                            
                            # Send to laboratory
                            self.send_data_to_lab(sensor_data)
                            
            except Exception as e:
                self.logger.error(f"BME680 thread error: {e}")
            time.sleep(1)

    def gps_thread(self):
        """Enhanced GPS monitoring thread with better error handling"""
        consecutive_errors = 0
        max_consecutive_errors = 10
    
        while True:
            try:
                if not self.gps_serial:
                    self.logger.warning("GPS serial not initialized, attempting reconnection...")
                    if self.setup_gps_serial():
                        consecutive_errors = 0
                    else:
                        time.sleep(5)
                        continue
                
                if self.gps_serial.in_waiting > 0:
                    try:
                        # Read with timeout
                        gps_line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                        
                        if gps_line:
                            # Reset error counter on successful read
                            consecutive_errors = 0
                            
                            # Parse the NMEA sentence
                            if self.parse_nmea_gps(gps_line):
                                # Successfully parsed GPS data
                                pass
                            else:
                                # Log unknown sentences occasionally for debugging
                                if gps_line.startswith(') and not any(x in gps_line for x in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC']):
                                    self.logger.debug(f"Unknown NMEA: {gps_line[:50]}...")
                                    
                    except UnicodeDecodeError:
                        self.logger.debug("GPS data decode error - possibly corrupted data")
                        consecutive_errors += 1
                        
                    except Exception as e:
                        self.logger.debug(f"GPS read error: {e}")
                        consecutive_errors += 1
                        
                else:
                    # No data waiting, short sleep
                    time.sleep(0.1)
                    
            except serial.SerialException as e:
                self.logger.error(f"GPS serial error: {e}")
                consecutive_errors += 1
                
                # Try to reconnect
                try:
                    if self.gps_serial:
                        self.gps_serial.close()
                    self.gps_serial = None
                    time.sleep(2)
                except:
                    pass
                    
            except Exception as e:
                self.logger.error(f"GPS thread error: {e}")
                consecutive_errors += 1
                
            # If too many consecutive errors, take a break
            if consecutive_errors >= max_consecutive_errors:
                self.logger.error(f"Too many GPS errors ({consecutive_errors}), pausing GPS thread")
                self.gps_fix = False
                time.sleep(30)  # Wait 30 seconds before trying again
                consecutive_errors = 0
                
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage

    def save_sensor_data(self, data):
        """Save sensor data to local CSV file"""
        try:
            if not self.log_file_path:
                date_str = datetime.now().strftime("%Y%m%d")
                filename = f"BME680_data_{date_str}.csv"
                self.log_file_path = os.path.join(
                    self.config['usb_mount_point'], filename
                )
                
                # Create file with headers if it doesn't exist
                if not os.path.exists(self.log_file_path):
                    with open(self.log_file_path, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            'timestamp', 'gps_lat', 'gps_lon', 'esp32_timestamp',
                            'temperature', 'humidity', 'pressure', 'gas_resistance'
                        ])
            
            # Append data
            with open(self.log_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    data['timestamp'], data['gps_lat'], data['gps_lon'],
                    data['esp32_timestamp'], data['temperature'], 
                    data['humidity'], data['pressure'], data['gas_resistance']
                ])
                
        except Exception as e:
            self.logger.error(f"Failed to save sensor data: {e}")

    def cleanup(self):
        """Clean shutdown of all systems"""
        self.logger.info("Shutting down platform...")
        
        # Stop motors
        self.emergency_stop()
        
        # Close serial connections
        if self.bme680_serial:
            self.bme680_serial.close()
            self.logger.info("BME680 serial closed")
        
        if self.gps_serial:
            self.gps_serial.close()
            self.logger.info("GPS serial closed")
        
        # Close lab connection
        if self.lab_socket:
            try:
                self.lab_socket.close()
                self.logger.info("Lab connection closed")
            except:
                pass
            self.lab_connection_active = False
        
        # GPIO cleanup
        try:
            GPIO.cleanup()
            self.logger.info("GPIO cleanup completed")
        except:
            pass
        


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Autonomous Environmental Monitoring Platform')
    parser.add_argument('--config', default='platform_config.json', 
                       help='Configuration file path')
    parser.add_argument('--test-motors', action='store_true',
                       help='Run motor test only')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       default='INFO', help='Set logging level')
    parser.add_argument('--set-home-now', action='store_true',
                       help='Set home coordinates to current GPS position at startup')
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Create platform instance
    platform = AutonomousPlatform(args.config)
    
    if args.test_motors:
        # Run motor test only
        if platform.test_motors_only():
            print("Motor test completed successfully")
            sys.exit(0)
        else:
            print("Motor test failed")
            sys.exit(1)
    else:
        # Run full platform
        try:
            platform.run()
        except KeyboardInterrupt:
            print("\nShutdown requested")
        except Exception as e:
            print(f"Platform error: {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()
