#!/usr/bin/env python3
"""
Laboratory Command Server for Autonomous Environmental Monitoring Platform
Enhanced to handle new platform features: home position management, mission acceptance/rejection,
heartbeats, file transfers, and improved communication protocols
"""

import socket
import json
import threading
import time
from datetime import datetime
import logging
import sys
import csv
import os

class LabServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.clients = {}
        self.data_log = []
        self.running = True
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('lab_server.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Create server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Data logging
        self.setup_data_logging()
        
        # File transfer handling
        self.file_transfer_dir = "received_files"
        os.makedirs(self.file_transfer_dir, exist_ok=True)
    
    def setup_data_logging(self):
        """Setup CSV logging for sensor data"""
        self.data_log_file = f"lab_data_{datetime.now().strftime('%Y%m%d')}.csv"
        if not os.path.exists(self.data_log_file):
            with open(self.data_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'platform_address', 'platform_id', 'gps_lat', 'gps_lon',
                    'temperature', 'humidity', 'pressure', 'gas_resistance',
                    'received_time'
                ])
    
    def send_command(self, client_socket, command):
        """Send command to platform"""
        try:
            message = json.dumps(command) + '\n'
            client_socket.send(message.encode())
            self.logger.info(f"Command sent: {command.get('type', 'UNKNOWN')} -> {command}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            return False
    
    def send_command_to_all(self, command):
        """Send command to all connected platforms"""
        success_count = 0
        for address, client_info in self.clients.items():
            if client_info.get('status') == 'online':
                if self.send_command(client_info['socket'], command):
                    success_count += 1
        return success_count
    
    def send_command_to_platform(self, platform_id, command):
        """Send command to specific platform by ID or index"""
        try:
            # Try to find by platform_id first
            target_client = None
            for address, client_info in self.clients.items():
                if client_info.get('platform_id') == platform_id:
                    target_client = client_info
                    break
            
            # If not found by ID, try by index
            if not target_client:
                try:
                    platform_addr = list(self.clients.keys())[platform_id]
                    target_client = self.clients[platform_addr]
                except (IndexError, KeyError):
                    pass
            
            if target_client and target_client.get('status') == 'online':
                return self.send_command(target_client['socket'], command)
            else:
                self.logger.error(f"Platform {platform_id} not found or offline")
                return False
                
        except Exception as e:
            self.logger.error(f"Error sending command to platform {platform_id}: {e}")
            return False
    
    def handle_file_transfer(self, client_socket, header):
        """Handle file transfer from platform"""
        try:
            filename = header['filename']
            file_size = header['size']
            timestamp = header.get('timestamp', datetime.now().isoformat())
            
            self.logger.info(f"Receiving file: {filename} ({file_size} bytes)")
            
            # Send acknowledgment
            client_socket.send(b'FILE_TRANSFER_ACK')
            
            # Receive file data
            safe_filename = filename.replace('/', '_').replace('\\', '_')
            file_path = os.path.join(self.file_transfer_dir, f"{timestamp[:10]}_{safe_filename}")
            
            bytes_received = 0
            with open(file_path, 'wb') as f:
                while bytes_received < file_size:
                    chunk_size = min(8192, file_size - bytes_received)
                    chunk = client_socket.recv(chunk_size)
                    if not chunk:
                        break
                    f.write(chunk)
                    bytes_received += len(chunk)
            
            if bytes_received == file_size:
                client_socket.send(b'FILE_TRANSFER_COMPLETE')
                self.logger.info(f"File transfer complete: {filename} -> {file_path}")
                return True
            else:
                client_socket.send(b'FILE_TRANSFER_ERROR')
                self.logger.error(f"File transfer incomplete: {bytes_received}/{file_size} bytes")
                return False
                
        except Exception as e:
            self.logger.error(f"File transfer error: {e}")
            try:
                client_socket.send(b'FILE_TRANSFER_ERROR')
            except:
                pass
            return False
    
    def handle_client(self, client_socket, address):
        """Handle individual client connection with enhanced protocol support"""
        self.logger.info(f"Platform connected from {address}")
        self.clients[address] = {
            'socket': client_socket,
            'last_seen': time.time(),
            'last_heartbeat': time.time(),
            'status': 'connected',
            'platform_status': {},
            'connected_time': datetime.now(),
            'platform_id': None,
            'mission_history': []
        }
        
        try:
            while self.running:
                # Set socket timeout for graceful shutdown
                client_socket.settimeout(2.0)
                
                try:
                    # Receive data from platform
                    data = client_socket.recv(4096)
                    if not data:
                        break
                    
                    # Handle multiple JSON messages in one packet
                    messages = data.decode().strip().split('\n')
                    for msg in messages:
                        if msg.strip():
                            try:
                                message = json.loads(msg)
                                self.handle_platform_message(address, message, client_socket)
                                self.clients[address]['last_seen'] = time.time()
                            except json.JSONDecodeError:
                                self.logger.warning(f"Invalid JSON from {address}: {msg[:100]}...")
                
                except socket.timeout:
                    # Check for heartbeat timeout
                    client_info = self.clients.get(address)
                    if client_info:
                        time_since_heartbeat = time.time() - client_info['last_heartbeat']
                        if time_since_heartbeat > 120:  # 2 minutes without heartbeat
                            self.logger.warning(f"Platform {address} heartbeat timeout ({time_since_heartbeat:.1f}s)")
                            client_info['status'] = 'timeout'
                    continue
                except Exception as e:
                    self.logger.error(f"Client handler error for {address}: {e}")
                    break
                
        finally:
            client_socket.close()
            if address in self.clients:
                self.clients[address]['status'] = 'disconnected'
                self.logger.info(f"Platform {address} disconnected")
    
    def handle_platform_message(self, address, data, client_socket):
        """Process messages received from platform with enhanced protocol support"""
        message_type = data.get('type', 'SENSOR_DATA')
        client_info = self.clients[address]
        
        if message_type == 'PLATFORM_CONNECTED':
            platform_id = data.get('platform_id', f"platform_{address[0]}_{address[1]}")
            client_info['platform_id'] = platform_id
            client_info['status'] = 'online'
            self.logger.info(f"✓ Platform {platform_id} came online from {address}")
            
        elif message_type == 'PLATFORM_DISCONNECTING':
            reason = data.get('reason', 'unknown')
            client_info['status'] = 'disconnecting'
            self.logger.info(f"Platform {address} disconnecting: {reason}")
            
        elif message_type == 'HEARTBEAT':
            client_info['last_heartbeat'] = time.time()
            client_info['status'] = 'online'
            # Heartbeats are logged at debug level to avoid spam
            self.logger.debug(f"Heartbeat from {client_info.get('platform_id', address)}")
            
        elif message_type == 'STATUS_UPDATE':
            client_info['platform_status'] = data
            client_info['status'] = 'online'
            position = data.get('position', {})
            mission = data.get('mission', {})
            home_status = data.get('home_status', {})
            speed_control = data.get('speed_control', {})
            
            platform_id = client_info.get('platform_id', str(address))
            self.logger.info(f"Status from {platform_id}: "
                           f"Pos=({position.get('lat', 0):.6f}, {position.get('lon', 0):.6f}), "
                           f"GPS={position.get('gps_fix', False)}, "
                           f"AtHome={home_status.get('at_home', False)}, "
                           f"Mission={mission.get('active', False)}, "
                           f"Speed={speed_control.get('current_limit', 0):.2f}")
                           
        elif message_type == 'MISSION_ACCEPTED':
            mission_type = data.get('mission_type', 'UNKNOWN')
            client_info['mission_history'].append({
                'type': 'ACCEPTED',
                'mission_type': mission_type,
                'timestamp': data.get('timestamp'),
                'details': data
            })
            platform_id = client_info.get('platform_id', str(address))
            self.logger.info(f"✓ Mission ACCEPTED by {platform_id}: {mission_type}")
            
        elif message_type == 'MISSION_REJECTED':
            reason = data.get('reason', 'unknown')
            client_info['mission_history'].append({
                'type': 'REJECTED',
                'reason': reason,
                'timestamp': data.get('timestamp'),
                'details': data
            })
            platform_id = client_info.get('platform_id', str(address))
            self.logger.warning(f"✗ Mission REJECTED by {platform_id}: {reason}")
            
        elif message_type == 'MISSION_COMPLETE':
            mission_summary = data.get('mission_summary', {})
            client_info['mission_history'].append({
                'type': 'COMPLETED',
                'timestamp': data.get('timestamp'),
                'summary': mission_summary
            })
            platform_id = client_info.get('platform_id', str(address))
            completed = mission_summary.get('completed_waypoints', 0)
            total = mission_summary.get('total_waypoints', 0)
            self.logger.info(f"✓ Mission COMPLETED by {platform_id}: {completed}/{total} waypoints")
            
        elif message_type == 'WAYPOINT_UPDATE':
            status = data.get('status', 'UNKNOWN')
            waypoint = data.get('waypoint', {})
            progress = data.get('mission_progress', {})
            platform_id = client_info.get('platform_id', str(address))
            self.logger.info(f"Waypoint {status} by {platform_id}: "
                           f"{progress.get('completed', 0)}/{progress.get('total', 0)} complete")
            
        elif message_type == 'EMERGENCY_STOP_CONFIRM':
            platform_id = client_info.get('platform_id', str(address))
            self.logger.warning(f"Emergency stop confirmed by {platform_id}")
            
        elif message_type == 'FILE_TRANSFER':
            # Handle file transfer
            self.handle_file_transfer(client_socket, data)
            return  # Don't process as regular data
            
        elif message_type in ['SPEED_SET_CONFIRM', 'SPEED_CHANGE_CONFIRM']:
            speed = data.get('speed', 0)
            action = data.get('action', 'SET')
            platform_id = client_info.get('platform_id', str(address))
            self.logger.info(f"Speed {action} confirmed by {platform_id}: {speed:.2f}")
            
        else:
            # Sensor data or unknown message type
            data['received_time'] = datetime.now().isoformat()
            data['platform_address'] = str(address)
            data['platform_id'] = client_info.get('platform_id', str(address))
            self.data_log.append(data)
            
            # Keep only last 1000 entries in memory
            if len(self.data_log) > 1000:
                self.data_log.pop(0)
            
            # Log to CSV file
            self.log_sensor_data(data)
            
            platform_id = client_info.get('platform_id', str(address))
            self.logger.info(f"Sensor data from {platform_id}: "
                           f"T={data.get('temperature', 'N/A')}°C, "
                           f"H={data.get('humidity', 'N/A')}%, "
                           f"P={data.get('pressure', 'N/A')}hPa, "
                           f"GPS=({data.get('gps_lat', 0):.6f}, {data.get('gps_lon', 0):.6f})")
    
    def log_sensor_data(self, data):
        """Log sensor data to CSV file"""
        try:
            with open(self.data_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    data.get('timestamp', ''),
                    data.get('platform_address', ''),
                    data.get('platform_id', ''),
                    data.get('gps_lat', 0),
                    data.get('gps_lon', 0),
                    data.get('temperature', 0),
                    data.get('humidity', 0),
                    data.get('pressure', 0),
                    data.get('gas_resistance', 0),
                    data.get('received_time', '')
                ])
        except Exception as e:
            self.logger.error(f"Failed to log sensor data: {e}")
    
    def list_platforms(self):
        """List all connected platforms with enhanced status information"""
        if not self.clients:
            print("No platforms connected")
            return
        
        print(f"\nConnected Platforms ({len(self.clients)}):")
        print("-" * 100)
        for i, (address, info) in enumerate(self.clients.items()):
            platform_status = info.get('platform_status', {})
            position = platform_status.get('position', {})
            mission = platform_status.get('mission', {})
            home_status = platform_status.get('home_status', {})
            speed_control = platform_status.get('speed_control', {})
            
            platform_id = info.get('platform_id', 'Unknown')
            status = info.get('status', 'unknown')
            
            # Status indicator
            status_icon = "✓" if status == 'online' else "✗" if status == 'disconnected' else "⚠"
            
            print(f"[{i}] {status_icon} {platform_id} ({address}) - {status.upper()}")
            print(f"    Connected: {info['connected_time'].strftime('%H:%M:%S')}")
            print(f"    Last seen: {time.time() - info['last_seen']:.1f}s ago")
            print(f"    Position: ({position.get('lat', 0):.6f}, {position.get('lon', 0):.6f})")
            print(f"    GPS Fix: {position.get('gps_fix', False)} | At Home: {home_status.get('at_home', False)}")
            print(f"    Mission Active: {mission.get('active', False)} | Returning Home: {mission.get('returning_home', False)}")
            if mission.get('active'):
                print(f"    Target: ({mission.get('target_lat', 0):.6f}, {mission.get('target_lon', 0):.6f})")
                waypoint_progress = mission.get('waypoint_progress')
                if waypoint_progress:
                    print(f"    Progress: {waypoint_progress.get('current', 0)}/{waypoint_progress.get('total', 0)} waypoints")
            print(f"    Speed Limit: {speed_control.get('current_limit', 0):.2f} (max: {speed_control.get('max_speed', 0):.2f})")
            
            # Show recent mission history
            recent_missions = info.get('mission_history', [])[-3:]  # Last 3 missions
            if recent_missions:
                print(f"    Recent missions: {len(recent_missions)} entries")
                for mission_record in recent_missions:
                    mission_type = mission_record.get('type', 'UNKNOWN')
                    timestamp = mission_record.get('timestamp', '')[:19]
                    if mission_type == 'REJECTED':
                        reason = mission_record.get('reason', 'unknown')
                        print(f"      {timestamp} - {mission_type}: {reason}")
                    else:
                        print(f"      {timestamp} - {mission_type}")
            print()
    
    def show_recent_data(self, count=10):
        """Show recent sensor data with platform IDs"""
        if not self.data_log:
            print("No sensor data available")
            return
        
        print(f"\nRecent Sensor Data (last {min(count, len(self.data_log))} entries):")
        print("-" * 120)
        print(f"{'Time':<20} {'Platform ID':<20} {'T(°C)':<8} {'H(%)':<8} {'P(hPa)':<10} {'Gas':<10} {'GPS':<25}")
        print("-" * 120)
        
        for data in self.data_log[-count:]:
            timestamp = data.get('timestamp', '')[:19]  # Remove microseconds
            platform_id = data.get('platform_id', 'Unknown')[:19]  # Truncate if too long
            temp = f"{data.get('temperature', 0):.1f}"
            humidity = f"{data.get('humidity', 0):.1f}"
            pressure = f"{data.get('pressure', 0):.1f}"
            gas = f"{data.get('gas_resistance', 0):.0f}"
            gps = f"({data.get('gps_lat', 0):.4f}, {data.get('gps_lon', 0):.4f})"
            
            print(f"{timestamp:<20} {platform_id:<20} {temp:<8} {humidity:<8} {pressure:<10} {gas:<10} {gps:<25}")
    
    def request_status_all(self):
        """Request status update from all platforms"""
        command = {'type': 'STATUS_REQUEST'}
        count = self.send_command_to_all(command)
        print(f"Status request sent to {count} platform(s)")
    
    def emergency_stop_all(self):
        """Send emergency stop to all platforms"""
        command = {'type': 'EMERGENCY_STOP'}
        count = self.send_command_to_all(command)
        print(f"EMERGENCY STOP sent to {count} platform(s)")
        self.logger.warning(f"Emergency stop command sent to {count} platforms")
    
    def return_home_all(self):
        """Send return home command to all platforms"""
        command = {'type': 'RETURN_HOME'}
        count = self.send_command_to_all(command)
        print(f"Return home command sent to {count} platform(s)")
    
    def goto_coordinates(self, lat, lon, platform_id=None):
        """Send goto command to platform(s)"""
        command = {
            'type': 'GOTO',
            'lat': float(lat),
            'lon': float(lon)
        }
        
        if platform_id is not None:
            success = self.send_command_to_platform(platform_id, command)
            if success:
                print(f"Goto command sent to platform {platform_id}")
                print("Note: Command will be rejected if platform is not at home")
            else:
                print(f"Failed to send command to platform {platform_id}")
        else:
            count = self.send_command_to_all(command)
            print(f"Goto command sent to {count} platform(s)")
            print("Note: Commands will be rejected by platforms not at home")
    
    def send_mission_path(self, waypoints, platform_id=None):
        """Send multi-waypoint mission to platform(s)"""
        command = {
            'type': 'MISSION_PATH',
            'waypoints': waypoints
        }
        
        if platform_id is not None:
            success = self.send_command_to_platform(platform_id, command)
            if success:
                print(f"Mission path ({len(waypoints)} waypoints) sent to platform {platform_id}")
            else:
                print(f"Failed to send mission to platform {platform_id}")
        else:
            count = self.send_command_to_all(command)
            print(f"Mission path sent to {count} platform(s)")
    
    def set_home_coordinates(self, lat, lon, platform_id=None):
        """Set home coordinates for platform(s)"""
        command = {
            'type': 'SET_HOME',
            'lat': float(lat),
            'lon': float(lon)
        }
        
        if platform_id is not None:
            success = self.send_command_to_platform(platform_id, command)
            if success:
                print(f"Home coordinates set for platform {platform_id}")
            else:
                print(f"Failed to set home for platform {platform_id}")
        else:
            count = self.send_command_to_all(command)
            print(f"Home coordinates set for {count} platform(s)")
    
    def set_speed(self, speed, platform_id=None):
        """Set speed limit for platform(s)"""
        command = {
            'type': 'SET_SPEED',
            'speed': float(speed)
        }
        
        if platform_id is not None:
            success = self.send_command_to_platform(platform_id, command)
            if success:
                print(f"Speed set to {speed} for platform {platform_id}")
            else:
                print(f"Failed to set speed for platform {platform_id}")
        else:
            count = self.send_command_to_all(command)
            print(f"Speed set to {speed} for {count} platform(s)")
    
    def adjust_speed(self, action, platform_id=None):
        """Increase or decrease speed for platform(s)"""
        command = {
            'type': 'INCREASE_SPEED' if action == 'increase' else 'DECREASE_SPEED'
        }
        
        if platform_id is not None:
            success = self.send_command_to_platform(platform_id, command)
            if success:
                print(f"Speed {action} command sent to platform {platform_id}")
            else:
                print(f"Failed to {action} speed for platform {platform_id}")
        else:
            count = self.send_command_to_all(command)
            print(f"Speed {action} command sent to {count} platform(s)")
    
    def command_interface(self):
        """Enhanced interactive command interface"""
        self.logger.info("Laboratory command interface started")
        print("\n" + "="*80)
        print("AUTONOMOUS PLATFORM LABORATORY COMMAND CENTER")
        print("="*80)
        print("Available commands:")
        print("  goto <lat> <lon> [platform_id] - Send platform to coordinates")
        print("  mission <lat1,lon1> <lat2,lon2> ... [platform_id] - Multi-waypoint mission")
        print("  home [platform_id] - Return platform to home")
        print("  sethome <lat> <lon> [platform_id] - Set home coordinates")
        print("  speed <value> [platform_id] - Set speed limit (0.0-1.0)")
        print("  faster [platform_id] - Increase speed")
        print("  slower [platform_id] - Decrease speed")
        print("  stop - Emergency stop all platforms")
        print("  status - Request status update from all platforms")
        print("  list - List connected platforms")
        print("  data [count] - Show recent sensor data")
        print("  files - List received files")
        print("  help - Show this help message")
        print("  quit - Exit server")
        print("="*80)
        print("Notes:")
        print("- Mission commands will be rejected if platform is not at home")
        print("- Platform will automatically return home after completing missions")
        print("- Heartbeat monitoring active (2-minute timeout)")
        print("="*80)
        
        while self.running:
            try:
                command_line = input("\nLab Command> ").strip()
                if not command_line:
                    continue
                
                parts = command_line.split()
                command = parts[0].lower()
                
                if command == 'quit':
                    self.running = False
                    break
                    
                elif command == 'help':
                    print("\nAvailable commands:")
                    print("  goto <lat> <lon> [platform_id] - Send platform to coordinates")
                    print("  mission <lat1,lon1> <lat2,lon2> ... [platform_id] - Multi-waypoint mission")
                    print("  home [platform_id] - Return platform to home")
                    print("  sethome <lat> <lon> [platform_id] - Set home coordinates")
                    print("  speed <value> [platform_id] - Set speed limit")
                    print("  faster/slower [platform_id] - Adjust speed")
                    print("  stop - Emergency stop all platforms")
                    print("  status - Request status update")
                    print("  list - List connected platforms")
                    print("  data [count] - Show recent sensor data")
                    print("  files - List received files")
                    print("  quit - Exit server")
                    
                elif command == 'list':
                    self.list_platforms()
                    
                elif command == 'data':
                    count = 10
                    if len(parts) > 1:
                        try:
                            count = int(parts[1])
                        except ValueError:
                            print("Invalid count, using default (10)")
                    self.show_recent_data(count)
                    
                elif command == 'files':
                    files = os.listdir(self.file_transfer_dir)
                    if files:
                        print(f"\nReceived files ({len(files)}):")
                        for file in sorted(files):
                            file_path = os.path.join(self.file_transfer_dir, file)
                            size = os.path.getsize(file_path)
                            mtime = datetime.fromtimestamp(os.path.getmtime(file_path))
                            print(f"  {file} ({size} bytes, {mtime.strftime('%Y-%m-%d %H:%M:%S')})")
                    else:
                        print("No files received yet")
                    
                elif command == 'status':
                    self.request_status_all()
                    
                elif command == 'stop':
                    confirm = input("Are you sure you want to emergency stop all platforms? (yes/no): ")
                    if confirm.lower() == 'yes':
                        self.emergency_stop_all()
                    else:
                        print("Emergency stop cancelled")
                        
                elif command in ['home', 'faster', 'slower']:
                    platform_id = None
                    if len(parts) > 1:
                        platform_id = parts[1]
                    
                    if command == 'home':
                        if platform_id is not None:
                            success = self.send_command_to_platform(platform_id, {'type': 'RETURN_HOME'})
                            if success:
                                print(f"Return home command sent to platform {platform_id}")
                        else:
                            self.return_home_all()
                    else:
                        action = 'increase' if command == 'faster' else 'decrease'
                        self.adjust_speed(action, platform_id)
                        
                elif command == 'goto':
                    if len(parts) < 3:
                        print("Usage: goto <lat> <lon> [platform_id]")
                        continue
                    
                    try:
                        lat = float(parts[1])
                        lon = float(parts[2])
                        platform_id = parts[3] if len(parts) > 3 else None
                        
                        self.goto_coordinates(lat, lon, platform_id)
                        
                    except ValueError:
                        print("Invalid coordinates")
                        
                elif command == 'mission':
                    if len(parts) < 2:
                        print("Usage: mission <lat1,lon1> <lat2,lon2> ... [platform_id]")
                        print("Example: mission 45.123,-63.456 45.124,-63.457 45.125,-63.458")
                        continue
                    
                    try:
                        waypoints = []
                        platform_id = None
                        
                        for part in parts[1:]:
                            if ',' in part:
                                lat_str, lon_str = part.split(',')
                                waypoints.append({
                                    'lat': float(lat_str),
                                    'lon': float(lon_str)
                                })
                            else:
                                # Assume it's platform_id if not a coordinate
                                platform_id = part
                        
                        if waypoints:
                            self.send_mission_path(waypoints, platform_id)
                        else:
                            print("No valid waypoints provided")
                        
                    except ValueError:
                        print("Invalid waypoint format. Use: lat,lon lat,lon ...")
                        
                elif command == 'sethome':
                    if len(parts) < 3:
                        print("Usage: sethome <lat> <lon> [platform_id]")
                        continue
                    
                    try:
                        lat = float(parts[1])
                        lon = float(parts[2])
                        platform_id = parts[3] if len(parts) > 3 else None
                        
                        self.set_home_coordinates(lat, lon, platform_id)
                        
                    except ValueError:
                        print("Invalid coordinates")
                        
                elif command == 'speed':
                    if len(parts) < 2:
                        print("Usage: speed <value> [platform_id]")
                        print("Speed value should be between 0.0 and 1.0")
                        continue
                    
                    try:
                        speed = float(parts[1])
                        platform_id = parts[2] if len(parts) > 2 else None
                        
                        if 0.0 <= speed <= 1.0:
                            self.set_speed(speed, platform_id)
                        else:
                            print("Speed must be between 0.0 and 1.0")
                        
                    except ValueError:
                        print("Invalid speed value")
                        
                else:
                    print(f"Unknown command: {command}. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                print("\nShutting down...")
                self.running = False
                break
            except Exception as e:
                self.logger.error(f"Command interface error: {e}")
    
    def start_server(self):
        """Start the laboratory server"""
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(10)  # Increased backlog for multiple platforms
            self.logger.info(f"Laboratory server listening on {self.host}:{self.port}")
            self.logger.info(f"File transfer directory: {os.path.abspath(self.file_transfer_dir)}")
            
            # Start command interface in separate thread
            command_thread = threading.Thread(target=self.command_interface, daemon=True)
            command_thread.start()
            
            while self.running:
                try:
                    self.server_socket.settimeout(1.0)
                    client_socket, address = self.server_socket.accept()
                    
                    # Handle client in separate thread
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, address),
                        daemon=True
                    )
                    client_thread.start()
                    
                except socket.timeout:
                    continue  # Normal timeout, continue loop
                except Exception as e:
                    if self.running:
                        self.logger.error(f"Server error: {e}")
                    
        except Exception as e:
            self.logger.error(f"Failed to start server: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up server resources"""
        self.logger.info("Shutting down laboratory server...")
        self.running = False
        
        # Close all client connections
        for address, client_info in list(self.clients.items()):
            try:
                client_info['socket'].close()
            except:
                pass
        
        # Close server socket
        try:
            self.server_socket.close()
        except:
            pass
        
        self.logger.info("Laboratory server shutdown complete")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Enhanced Laboratory Command Server')
    parser.add_argument('--host', default='0.0.0.0', help='Server host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8080, help='Server port (default: 8080)')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       default='INFO', help='Set logging level')
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    print("Enhanced Laboratory Command Server")
    print("=================================")
    print("New features:")
    print("- Home position management and validation")
    print("- Mission acceptance/rejection handling")
    print("- Multi-waypoint mission support")
    print("- Speed control commands")
    print("- File transfer support")
    print("- Heartbeat monitoring")
    print("- Enhanced status reporting")
    print("=================================")
    
    # Create and start server
    server = LabServer(args.host, args.port)
    
    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\nServer interrupted by user")
    finally:
        server.cleanup()

if __name__ == "__main__":
    main()