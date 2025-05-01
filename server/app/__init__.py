"""
Robotic Control System with Kinect Integration
Version: 2.0
Features:
- Kinect v1 control with libfreenect
- Webcam fallback mechanism
- Serial communication management
- Real-time GPS tracking
- Web-based control interface
- Authentication system
- Multi-threaded architecture
"""

# --------------------------
# Import Section
# --------------------------
import os
import re
import json
import time
import threading
import platform
from threading import Lock
import cv2
import numpy as np
import serial
import geocoder
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, render_template, request
from flask_socketio import SocketIO
from flask_httpauth import HTTPBasicAuth
import signal
import sys

# --------------------------
# Configuration Setup
# --------------------------
load_dotenv()  # Load environment variables from .env file

# --------------------------
# Flask Application Initialization
# --------------------------
app = Flask(__name__, template_folder='../templates', static_folder='../static')
app.config['SECRET_KEY'] = os.getenv('FLASK_SECRET', 'supersecretkey')
auth = HTTPBasicAuth()
socketio = SocketIO(app, 
                   async_mode='threading', 
                   cors_allowed_origins="*", 
                   engineio_logger=False)
# --------------------------
# Conditional Kinect Import
# --------------------------
KINECT_AVAILABLE = False
try:
    import freenect
    KINECT_AVAILABLE = True
except ImportError:
    print("Kinect library not found, using webcam fallback")
except Exception as e:
    print(f"Kinect initialization error: {e}")
# --------------------------
# Hardware Manager: Serial Communication
# --------------------------
class SerialManager:
    """
    Manages serial communication with Arduino-based controllers
    Features:
    - Automatic port detection
    - Retry connection logic
    - Telemetry parsing
    - GPS data handling
    """
    
    def __init__(self):
        """Initialize serial connection and related parameters"""
        self.ser = None              # Serial connection object
        self.lock = Lock()           # Thread lock for serial operations
        self.last_data = {}          # Last received telemetry data
        self.gps_coords = {          # GPS coordinates storage
            'lat': 0.0, 
            'lon': 0.0, 
            'source': 'default',
            'accuracy': 0.0
        }
        self._connect_serial()       # Establish initial connection
        self.update_interval = 0.1   # Serial read interval in seconds

        self.ibus_ser = None  # Separate serial for IBus receiver
        self.ibus_lock = Lock()
        self.last_update = 0
        self.channels = [0] * 10  # Store all 10 channels

    def _get_serial_config(self):
        """
        Get serial configuration based on OS and environment variables
        Returns:
            tuple: (port, baud_rate)
        """
        port = os.getenv('SERIAL_PORT', self._default_port())
        baud = int(os.getenv('SERIAL_BAUD', '115200'))
        return port, baud

    def _default_port(self):
        """Get default serial port based on OS"""
        return {
            'Linux': '/dev/ttyACM0',
            'Windows': 'COM6',
            'Darwin': '/dev/cu.usbmodem14101'
        }.get(platform.system(), '/dev/ttyACM0')

    def _connect_serial(self):
        """Establish serial connection with retry logic"""
        port, baud = self._get_serial_config()
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=1,
                    write_timeout=1
                )
                print(f"Connected to serial on {port} @ {baud}bps")
                return
            except serial.SerialException as e:
                if attempt == max_retries - 1:
                    print(f"Serial connection failed: {e}")
                    self._print_port_help()
                    raise RuntimeError("Serial connection failed") from e
                print(f"Connection attempt {attempt+1} failed, retrying...")
                time.sleep(2 ** attempt)
    
    def parse_telemetry(self, data):
        """
        Parse incoming telemetry data with support for multiple formats
        Args:
            data (str): Raw serial data string
        Returns:
            dict: Parsed telemetry values
        """
        parsed = {}
        try:
            # Attempt JSON parsing first
            if json_match := re.search(r'\{.*\}', data):
                parsed.update(json.loads(json_match.group()))
                data = data.replace(json_match.group(), '')

            # Parse remaining key-value pairs
            for part in filter(None, re.split(r'[, \t]+', data.strip())):
                if ':' in part:
                    key, value = part.split(':', 1)
                elif '=' in part:
                    key, value = part.split('=', 1)
                else:
                    continue
                
                key = key.strip().lower()
                value = value.strip()
                if key:
                    parsed[key] = self._parse_value(value)

            # Update GPS coordinates if available
            self._update_gps(parsed)
            
        except (json.JSONDecodeError, ValueError) as e:
            print(f"Telemetry parsing error: {e}")
        return parsed

    def _parse_value(self, value):
        """
        Convert string values to appropriate data types
        Args:
            value (str): Raw value string
        Returns:
            int/float/str: Converted value
        """
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value

    def _update_gps(self, data):
        """Update GPS coordinates from parsed data"""
        if 'gps_lat' in data and 'gps_lon' in data:
            self.gps_coords.update({
                'lat': float(data['gps_lat']),
                'lon': float(data['gps_lon']),
                'source': 'gps_module',
                'accuracy': float(data.get('gps_acc', 0.0))
            })
        elif 'lat' in data and 'lon' in data:
            self.gps_coords.update({
                'lat': float(data['lat']),
                'lon': float(data['lon']),
                'source': 'serial',
                'accuracy': 5.0  # Default serial accuracy
            })

    def get_gps(self):
        """
        Get current GPS coordinates with IP fallback
        Returns:
            dict: GPS coordinates and metadata
        """
        if self.gps_coords['source'] == 'default':
            try:
                if (loc := geocoder.ip('me')).ok:
                    self.gps_coords.update({
                        'lat': loc.latlng[0],
                        'lon': loc.latlng[1],
                        'source': 'ip_geolocation',
                        'accuracy': 500.0  # IP geolocation accuracy
                    })
            except Exception as e:
                print(f"Geocoding failed: {e}")
        return self.gps_coords
    
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
        """
        Send command to connected serial device
        Args:
            cmd (str): Command string to send
        Returns:
            bool: True if successful
        """
        with self.lock:
            try:
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
        if self.ser:
            self.ser.close()
        self._connect_serial()

    def _print_port_help(self):
        """Display port troubleshooting info"""
        print("\nAvailable ports:")
        if platform.system() == 'Windows':
            print("Check Device Manager -> Ports (COM & LPT)")
        else:
            print("Common ports: /dev/ttyACM*, /dev/ttyUSB*, /dev/ttyS*")

# --------------------------
# Vision Manager: Kinect & Webcam
# --------------------------

class VisionManager:
    """
    Manages vision inputs with multiple fallback layers:
    1. Try Kinect hardware if library available
    2. Try system default webcam
    3. Generate test pattern as final fallback
    """    
    def __init__(self):
        self.lock = Lock()
        self.rgb_frame = None
        self.depth_frame = None
        self.running = False
        self.using_kinect = False
        self.camera = None
        self._initialize_vision()

    def _initialize_vision(self):
        """Multi-stage vision system initialization"""
        try:
            if KINECT_AVAILABLE:
                self._try_init_kinect()
            
            if not self.using_kinect:
                self._try_init_webcam()
                
        except Exception as e:
            print(f"Vision init error: {e}")
            self._setup_test_pattern()

    def _try_init_kinect(self):
        """Attempt Kinect initialization with safety checks"""
        if not KINECT_AVAILABLE:
            return

        try:
            self.ctx = freenect.init()
            if freenect.num_devices(self.ctx) > 0:
                self.dev = freenect.open_device(self.ctx, 0)
                freenect.start_video(self.dev)
                freenect.start_depth(self.dev)
                self.using_kinect = True
                self.running = True
                threading.Thread(target=self._kinect_update_loop).start()
                print("Kinect initialized successfully")
        except Exception as e:
            print(f"Kinect hardware error: {e}")
            self._cleanup_kinect()
        threading.Thread(target=self._kinect_update_loop, daemon=True).start()
    def _try_init_webcam(self):
        """Initialize webcam with multiple attempt strategies"""
        for i in range(4):  # Try first 4 video devices
            try:
                self.camera = cv2.VideoCapture(i)
                if self.camera.isOpened():
                    self.running = True
                    threading.Thread(target=self._webcam_update_loop).start()
                    print(f"Webcam initialized on index {i}")
                    return
            except Exception as e:
                print(f"Webcam init attempt {i} failed: {e}")
        
        print("No webcams found, using test pattern")
        self._setup_test_pattern()
        threading.Thread(target=self._webcam_update_loop, daemon=True).start()

    def _setup_test_pattern(self):
        """Final fallback to generated test pattern"""
        self.running = True
        threading.Thread(target=self._test_pattern_loop, daemon=True).start()
        print("Using test pattern fallback")

    def _kinect_update_loop(self):
        """Kinect frame update thread"""
        try:
            while self.running and KINECT_AVAILABLE:
                freenect.process_events(self.ctx)
                with self.lock:
                    self.rgb_frame = freenect.sync_get_video(self.dev)[0]
                    self.depth_frame = freenect.sync_get_depth(self.dev)[0]
                time.sleep(0.03)
        except Exception as e:
            print(f"Kinect loop failed: {e}")
            self._cleanup_kinect()
            self._try_init_webcam()

    def _webcam_update_loop(self):
        """Webcam frame update thread"""
        try:
            while self.running and self.camera:
                ret, frame = self.camera.read()
                if ret:
                    with self.lock:
                        self.rgb_frame = frame
                time.sleep(0.03)
        except Exception as e:
            print(f"Webcam loop failed: {e}")
            self._setup_test_pattern()

    def _test_pattern_loop(self):
        """Test pattern generation thread"""
        while self.running:
            with self.lock:
                self.rgb_frame = self._generate_test_pattern()
            time.sleep(0.1)

    def get_rgb_frame(self):
        """Get current RGB frame with multiple fallback sources"""
        with self.lock:
            if self.rgb_frame is not None:
                frame = self.rgb_frame.copy()
                if self.using_kinect:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                frame = self._generate_test_pattern()
            
            frame = cv2.resize(frame, (640, 480))
            _, jpeg = cv2.imencode('.jpg', frame)
            return jpeg.tobytes()

    def get_depth_frame(self):
        """Get depth frame or placeholder if unavailable"""
        with self.lock:
            if self.using_kinect and KINECT_AVAILABLE and self.depth_frame is not None:
                depth = (self.depth_frame >> 3).astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
            else:
                depth_colored = self._generate_test_pattern(text="Depth Unavailable")
            
            _, jpeg = cv2.imencode('.jpg', depth_colored)
            return jpeg.tobytes()

    def _generate_test_pattern(self, text="No Video Input"):
        """Generate diagnostic test pattern"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, text, (150, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        return frame

    def _cleanup_kinect(self):
        """Safe Kinect resource cleanup"""
        if KINECT_AVAILABLE:
            try:
                if hasattr(self, 'dev'):
                    freenect.close_device(self.dev)
                if hasattr(self, 'ctx'):
                    freenect.shutdown(self.ctx)
            except Exception as e:
                print(f"Kinect cleanup error: {e}")
        self.using_kinect = False

    def __del__(self):
        """Resource cleanup"""
        self.running = False
        self._cleanup_kinect()
        if self.camera:
            self.camera.release()
# --------------------------
# Initialize System Components
# --------------------------
serial_mgr = SerialManager()
vision_mgr = VisionManager()

# --------------------------
# Web Application Routes
# --------------------------
@app.route('/')
@auth.login_required
def control_dashboard():
    """Serve main control interface"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """RGB video streaming endpoint"""
    def generate():
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + 
                   vision_mgr.get_rgb_frame() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    """Depth video streaming endpoint"""
    def generate():
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + 
                   vision_mgr.get_depth_frame() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gps_data')
def gps_data():
    """Current GPS coordinates endpoint"""
    return jsonify(serial_mgr.get_gps())

@app.route('/telemetry')
def telemetry_data():
    """Current telemetry data endpoint"""
    return jsonify(serial_mgr.last_data)

@app.route('/kinect/control', methods=['POST'])
@auth.login_required
def kinect_control():
    """Kinect control command endpoint"""
    data = request.get_json()
    if not data or 'command' not in data:
        return jsonify({'error': 'Invalid request'}), 400
    
    success = vision_mgr.control_kinect(data['command'], data.get('value'))
    return jsonify({'success': success})

# --------------------------
# WebSocket Handlers
# --------------------------
@socketio.on('connect')
def handle_connect():
    """Handle new WebSocket connections"""
    print(f"Client connected: {request.sid}")
    threading.Thread(target=background_data_push).start()

def background_data_push():
    """Push periodic updates to connected clients"""
    while vision_mgr.running:  # Use the running flag as condition
        if data := serial_mgr.read_data():
            socketio.emit('arduino_data_update', data)
            socketio.emit('gps_update', serial_mgr.get_gps())
        time.sleep(0.1)
        # // Example Serial Telemetry JSON Structure : {
        # //   "t" : 123456, // Timestamp in ms
        # //   "ch_raw" : [  // Raw channels as objects
        # //     {"ch1" : 1500}, {"ch2" : 1490}, {"ch3" : 1520}, ...
        # //   ],
        # //   "sw" : [ 0, 1, 0, 1 ], // Switch states for channels 5–8
        # //   "lights" : true,
        # //   "estop" : false,
        # //   "brakes" : false,
        # //   "motors" : [ 120, -90 ], // Motor1 & Motor2 PWM speeds
        # //   "status" : "MOVING"
        # // }

@socketio.on('control_command')
def handle_control_command(cmd):
    """Handle control commands from web interface"""
    print(f"Received command: {cmd}")
    if cmd.startswith('drive:'):
        parts = cmd[6:].split(',')
        if len(parts) == 2:
            serial_mgr.send_command(cmd)
    else:
        serial_mgr.send_command(cmd)

# --------------------------
# Authentication System
# --------------------------
@auth.verify_password
def verify_credentials(username, password):
    """Basic authentication verification"""
    return (username == os.getenv('WEB_USER', 'admin') and
            password == os.getenv('WEB_PASS', 'secret'))

# --------------------------
# Template Helpers
# --------------------------
@app.context_processor
def inject_helpers():
    """Inject template helper functions"""
    return {
        'format_key': lambda k: k.replace('_', ' ').title(),
        'format_value': lambda v: f"{v:.2f}" if isinstance(v, float) else v
    }

# Add this before the __main__ block
def handle_shutdown(signum, frame):
    print("\nShutting down gracefully...")
    vision_mgr.running = False
    if serial_mgr.ser and serial_mgr.ser.is_open:
        serial_mgr.ser.close()
    sys.exit(0)

# Register signal handlers
signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)
# --------------------------
# Main Application Entry
# --------------------------

# ================= Main Execution =================
if __name__ == '__main__':
    # socketio.run(app, host='0.0.0.0', port=5001, debug=True)
    try:
        socketio.run(
            app,
            host=os.getenv('HOST', '0.0.0.0'),
            port=int(os.getenv('PORT', '5000')),
            debug=os.getenv('DEBUG', 'false').lower() == 'true',
            allow_unsafe_werkzeug=True
        )
    except KeyboardInterrupt:
        handle_shutdown(None, None)