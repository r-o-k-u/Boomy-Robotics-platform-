import sys
import platform
import threading
import time
import os
import json
import re
from threading import Lock
from flask import Flask, Response, jsonify, render_template
from flask_socketio import SocketIO
from flask_httpauth import HTTPBasicAuth
import requests
import serial
import numpy as np
import cv2
from dotenv import load_dotenv

import geocoder
g = geocoder.ip('me')
print(g.latlng[0], g.latlng[1])

# Load environment variables
load_dotenv()

# Initialize Flask app
app = Flask(__name__)
auth = HTTPBasicAuth()
socketio = SocketIO(app, async_mode='threading', engineio_logger=False)

# --------------------------
# Hardware Configuration
# --------------------------

class SerialManager:
    """Manages serial communication with Arduino"""
    def __init__(self):
        self.ser = None
        self.ibus_ser = None  # Separate serial for IBus receiver
        self.lock = Lock()
        self.ibus_lock = Lock()
        self.update_interval = 0.1  # 100ms
        self.last_update = 0
        self.last_data = {}
        self.gps_coords = {'lat': 0.0, 'lon': 0.0, 'source': 'default'}
        self.channels = [0] * 10  # Store all 10 channels
        self.connect()

    def get_serial_port(self, env_var='SERIAL_PORT'):
        """Get serial port from environment variables"""
        return os.getenv(env_var, self.get_default_port())

    def get_default_port(self):
        """Get default serial port based on OS"""
        return {
            'Linux': '/dev/ttyACM0',
            'Windows': 'COM6',
            'Darwin': '/dev/cu.usbmodem14101'
        }.get(platform.system(), '/dev/ttyACM0')

    def connect(self):
        """Establish serial connections with retry logic"""
        # Main serial connection
        self.connect_serial('SERIAL_PORT', 'SERIAL_BAUD')

        # IBus receiver connection
        # self.connect_serial('IBUS_PORT', 'IBUS_BAUD', is_ibus=True)

    def connect_serial(self, port_env, baud_env, is_ibus=False):
        """Generic serial connection helper"""
        max_retries = 3
        port = self.get_serial_port(port_env)
        baud = int(os.getenv(baud_env, '115200'))

        for attempt in range(max_retries):
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=1,
                    write_timeout=1
                )
                if is_ibus:
                    self.ibus_ser = ser
                    print(f"Connected to IBus receiver on {port} at {baud}bps")
                else:
                    self.ser = ser
                    print(f"Connected to main serial on {port} at {baud}bps")
                return
            except serial.SerialException as e:
                if attempt == max_retries - 1:
                    print(f"Serial connection failed: {e}")
                    self.print_port_help()
                    sys.exit(1)
                print(f"Connection attempt {attempt+1} failed, retrying...")
                time.sleep(2 ** attempt)

    def print_port_help(self):
        """Display port troubleshooting info"""
        print("\nAvailable ports:")
        if platform.system() == 'Windows':
            print("Check Device Manager -> Ports (COM & LPT)")
        else:
            print("Common ports: /dev/ttyACM*, /dev/ttyUSB*, /dev/ttyS*")

    def get_gps(self):
        """Get current GPS coordinates from the best available source."""
        # If GPS coordinates from the serial manager are available, return those
        if self.gps_coords['source'] != 'default': 
            return self.gps_coords

        # Fallback to IP geolocation if no GPS data
        try:
            ip_loc = geocoder.ip('me')
            if ip_loc.ok:
                self.gps_coords = {
                    'lat': ip_loc.latlng[0],
                    'lon': ip_loc.latlng[1],
                    'source': 'ip_geolocation'
                }
                return self.gps_coords
        except Exception as e:
            print(f"IP geolocation failed: {e}")
        
        # Default case: if everything fails
        return {'lat': 0.0, 'lon': 0.0, 'source': 'default'}
    
    # def parse_telemetry(self, data):
    #     """Parse telemetry data into dictionary"""
    #     try:
    #         # Handle JSON formatted data
    #         if data.startswith('{') and data.endswith('}'):
    #             parsed = json.loads(data)
    #             self._update_gps_from_telemetry(parsed)
    #             return parsed

    #         # Handle key=value pairs
    #         parsed = {}
    #         for pair in data.split(','):
    #             if '=' in pair:
    #                 key, value = pair.split('=', 1)
    #                 key = key.strip().lower()
    #                 parsed[key] = self._parse_value(value.strip())

    #         self._update_gps_from_telemetry(parsed)
    #         self._log_receiver_channels(parsed)
    #         return parsed

    #     except Exception as e:
    #         print(f"Error parsing telemetry: {e}")
    #         return {}
    def parse_telemetry(self, data):
        """Parse telemetry data into dictionary"""
        parsed = {}
        try:
            # First try to extract JSON data from the string
            json_match = re.search(r'\{.*\}', data)
            if json_match:
                json_str = json_match.group()
                try:
                    json_data = json.loads(json_str)
                    parsed.update(json_data)
                    # Remove JSON part from the original string
                    data = data.replace(json_str, '')
                except json.JSONDecodeError as e:
                    print(f"JSON parse error: {e}")

            # Now parse remaining key-value pairs (support both ':' and '=' separators)
            # Split on both commas and whitespace for different formats
            for part in re.split(r'[, \t]+', data.strip()):
                if not part:
                    continue
                # Split on first occurrence of either : or =
                if ':' in part:
                    key, value = part.split(':', 1)
                elif '=' in part:
                    key, value = part.split('=', 1)
                else:
                    continue

                key = key.strip().lower()
                value = value.strip()
                if key:  # Skip empty keys
                    parsed[key] = value #""" self._parse_value(value) """

            # Update derived data
            self._update_gps_from_telemetry(parsed)
           # self._log_receiver_channels(parsed)

        except Exception as e:
            print(f"Error parsing telemetry: {e}")

        return parsed

    def _update_gps_from_telemetry(self, data):
        """Update GPS coordinates from telemetry data"""
        if 'gps_lat' in data and 'gps_lon' in data:
            self.gps_coords = {
                'lat': float(data['gps_lat']),
                'lon': float(data['gps_lon']),
                'source': 'gps_module'
            }
        elif 'lat' in data and 'lon' in data:
            self.gps_coords = {
                'lat': float(data['lat']),
                'lon': float(data['lon']),
                'source': 'serial'
            }

    def _log_receiver_channels(self, data):
        """Log receiver channel values to console"""
        channels = {k: v for k, v in data.items() if k.startswith('ch')}
        if channels:
            print("Receiver Channels:", json.dumps(channels, indent=2))

    def read_data(self):
        """Read data from serial port with rate limiting"""
        with self.lock:
            now = time.time()
            if now - self.last_update >= self.update_interval:
                try:
                    if self.ser.in_waiting:
                        data = self.ser.readline().decode().strip()
                        self.last_update = now
                        parsed = self.parse_telemetry(data)
                        if parsed:
                            self.last_data.update(parsed)
                            return self.last_data
                except (serial.SerialException, UnicodeDecodeError) as e:
                    print(f"Serial error: {e}")
                    self.reconnect()
        return None

    def send_command(self, cmd):
        """Send command to Arduino with error handling"""
        with self.lock:
            try:
                # Process drive commands specially
                if cmd.startswith('drive:'):
                    left, right = cmd[6:].split(',')
                    cmd = f"MOTOR:{left},{right}"

                self.ser.write(f"{cmd}\n".encode())
                print(f"Sent command: {cmd}")
                return True
            except serial.SerialException as e:
                print(f"Send failed: {e}")
                self.reconnect()
                return False

    def reconnect(self):
        """Handle serial reconnection"""
        print("Attempting reconnect...")
        self.ser.close()
        time.sleep(1)
        self.connect()

class CameraManager:
    """Manages camera feeds (Kinect/Webcam)"""
    def __init__(self):
        self.lock = Lock()
        self.frame = None
        self.running = False
        self.kinect_ctx = None
        self.camera = None
        self.init_camera()

    def init_camera(self):
        """Initialize appropriate camera based on platform"""
        try:
            if self._is_kinect_available():
                self._init_kinect()
            else:
                self._init_webcam()

            self.running = True
            threading.Thread(target=self._update_frame, daemon=True).start()

        except Exception as e:
            print(f"Camera init error: {e}")
            self.camera = None

    def _is_kinect_available(self):
        """Check for Kinect availability"""
        if platform.system() == 'Linux' and 'arm' in platform.machine().lower():
            try:
                global freenect
                import freenect
                return True
            except ImportError:
                print("freenect library not installed, using webcam")
        return False

    def _init_kinect(self):
        """Initialize Kinect sensor"""
        self.kinect_ctx = freenect.init()
        freenect.set_led(self.kinect_ctx, freenect.LED_GREEN)
        print("Initialized Kinect sensor")

    def _init_webcam(self):
        """Initialize webcam with auto-detection"""
        for i in range(4):  # Try first 4 indices
            self.camera = cv2.VideoCapture(i)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                print(f"Initialized webcam on index {i}")
                return
        raise RuntimeError("No webcam found")

    def _update_frame(self):
        """Main frame update loop"""
        while self.running:
            try:
                if self.kinect_ctx:
                    freenect.runloop(video=self._kinect_callback)
                elif self.camera:
                    self._read_webcam_frame()
            except Exception as e:
                print(f"Frame update error: {e}")
                time.sleep(1)

    def _kinect_callback(self, dev, frame, timestamp):
        """Kinect video frame callback"""
        with self.lock:
            self.frame = frame.copy()

    def _read_webcam_frame(self):
        """Read frame from webcam"""
        ret, frame = self.camera.read()
        if ret:
            with self.lock:
                self.frame = frame
        else:
            print("Webcam read failed, attempting reconnect...")
            self.init_camera()
            time.sleep(1)

    def get_frame(self):
        """Get current frame as JPEG bytes"""
        with self.lock:
            frame = self.frame if self.frame is not None else self._test_pattern()

            if self.kinect_ctx:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame = cv2.resize(frame, (640, 480))
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return jpeg.tobytes()

    def _test_pattern(self):
        """Generate test pattern frame"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "No Camera Feed", (320, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        return frame

    def __del__(self):
        """Cleanup resources"""
        self.running = False
        if self.camera:
            self.camera.release()
        if self.kinect_ctx:
            freenect.safe_shutdown(self.kinect_ctx)

# --------------------------
# Web Application Setup
# --------------------------

serial_mgr = SerialManager()
camera_manager = CameraManager()

@app.context_processor
def inject_helpers():
    """Inject template helpers"""
    return {
        'formatKey': lambda k: k.replace('_', ' ').title(),
        'formatValue': lambda v: f"{float(v):.2f}" if v.replace('.','').isdigit() else v
    }

@auth.verify_password
def verify_password(username, password):
    """Basic authentication handler"""
    return username == os.getenv('WEB_USER', 'admin') and \
           password == os.getenv('WEB_PASS', 'secret')

def generate_frames():
    """Video streaming generator"""
    while True:
        frame = camera_manager.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


def get_gps(self):
    """Get current GPS coordinates from best available source"""
    # Fallback to IP geolocation if no GPS data
    if self.gps_coords['source'] in ['none', 'ip_geolocation']:
        try:
            ip_loc = geocoder.ip('me')
            if ip_loc.ok:
                self.gps_coords = {
                    'lat': ip_loc.latlng[0],
                    'lon': ip_loc.latlng[1],
                    'source': 'ip_geolocation'
                }
        except Exception as e:
            print(f"IP geolocation failed: {e}")

    return self.gps_coords

def background_listener():
    """Background thread for serial data"""
    while True:
        # print("Listening for serial data...")
        if data := serial_mgr.read_data():
            # print(f"Received data: {data}")
            # Always emit telemetry data
            socketio.emit('receiver_channels', data)

            # Get current GPS coordinates from best source
            gps = serial_mgr.get_gps()
            socketio.emit('gps_update', {
                'lat': gps['lat'],
                'lon': gps['lon'],
                'source': gps['source']
            })

            time.sleep(0.05)  # Prevent flooding


@app.route('/')
@auth.login_required
def index():
    """Main control interface"""
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gps_data')
def gps_data():
    """Current GPS coordinates from best available source"""
    gps = serial_mgr.get_gps()
    return jsonify({
        'lat': gps['lat'],
        'lon': gps['lon'],
        'source': gps['source']
    })


@app.route('/receiver_data')
def receiver_data():
    """Current receiver channel values"""
    channels = {k: v for k, v in serial_mgr.last_data.items() if k.startswith('ch')}
    return jsonify(channels)


@app.route('/set_gps/<float:lat>/<float:lon>')
@auth.login_required
def set_gps(lat, lon):
    """Manually set GPS coordinates"""
    serial_mgr.gps_coords = {
        'lat': lat,
        'lon': lon,
        'source': 'manual'
    }
    return jsonify({
        'status': 'success',
        'message': f'GPS coordinates set to {lat}, {lon}'
    })

@socketio.on('connect')
def handle_connect():
    """WebSocket connect handler"""
    print("Client connected")
    socketio.start_background_task(background_listener)

@socketio.on('control_command')
def handle_command(cmd):
    """Handle control commands from web interface"""
    print(f"Received command: {cmd}")

    # Special handling for drive commands
    if cmd.startswith('drive:'):
        # Format: "drive:left_power,right_power"
        if serial_mgr.send_command(cmd):
            # Update last data with motor values
            left, right = cmd[6:].split(',')
            serial_mgr.last_data.update({
                'left_motor': left,
                'right_motor': right
            })
    else:
        # Standard command pass-through
        serial_mgr.send_command(cmd)

if __name__ == '__main__':
    socketio.run(
        app,
        host=os.getenv('HOST', '0.0.0.0'),
        port=int(os.getenv('PORT', '5000')),
        debug=os.getenv('DEBUG', 'false').lower() == 'true',
        allow_unsafe_werkzeug=True
    )
