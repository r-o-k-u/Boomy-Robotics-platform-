import sys
import platform
import threading
import time
import os
from threading import Lock
from flask import Flask, Response, jsonify, render_template
from flask_socketio import SocketIO
from flask_httpauth import HTTPBasicAuth
import serial
import numpy as np
import cv2
from dotenv import load_dotenv

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
        self.lock = Lock()
        self.update_interval = 0.1  # 100ms
        self.last_update = 0
        self.connect()

    def get_serial_port(self):
        """Get default serial port based on OS"""
        return {
            'Linux': '/dev/ttyACM0',
            'Windows': 'COM5',
            'Darwin': '/dev/cu.usbmodem14101'
        }.get(platform.system(), os.getenv('SERIAL_PORT', '/dev/ttyACM0'))

    def connect(self):
        """Establish serial connection with retry logic"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(
                    port=self.get_serial_port(),
                    baudrate=int(os.getenv('SERIAL_BAUD', '115200')),
                    timeout=1,
                    write_timeout=1
                )
                print(f"Connected to {self.ser.port} at {self.ser.baudrate}bps")
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

    def read_data(self):
        """Read data from serial port with rate limiting"""
        with self.lock:
            now = time.time()
            if now - self.last_update >= self.update_interval:
                try:
                    if self.ser.in_waiting:
                        data = self.ser.readline().decode().strip()
                        self.last_update = now
                        return data
                except (serial.SerialException, UnicodeDecodeError) as e:
                    print(f"Serial error: {e}")
                    self.reconnect()
        return None

    def send_command(self, cmd):
        """Send command to Arduino with error handling"""
        with self.lock:
            try:
                self.ser.write(f"{cmd}\n".encode())
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
        self.camera = cv2.VideoCapture(self._find_webcam_index())
        if not self.camera.isOpened():
            raise RuntimeError("Webcam not found")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print(f"Initialized webcam on index {self._find_webcam_index()}")

    def _find_webcam_index(self):
        """Detect available webcam index"""
        for i in range(4):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                cap.release()
                return i
        return 0

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
            time.sleep(0.1)

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
        cv2.putText(frame, "No Camera Feed", (100, 240), 
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
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + 
               camera_manager.get_frame() + b'\r\n')

def background_listener():
    """Background thread for serial data"""
    while True:
        if data := serial_mgr.read_data():
            socketio.emit('arduino_data', {'data': data})

@app.route('/')
@auth.login_required
def index():
    """Main control interface"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Define default GPS coordinates
current_gps_latitude = -0.717178
current_gps_longitude =36.431026

@app.route('/gps_data')
def gps_data():
    return jsonify({
        'lat': current_gps_latitude,
        'lon': current_gps_longitude
    })
@socketio.on('connect')
def handle_connect():
    """WebSocket connect handler"""
    socketio.start_background_task(background_listener)

@socketio.on('control_command')
def handle_command(cmd):
    """Handle control commands"""
    if serial_mgr.send_command(cmd):
        print(f"Command sent: {cmd}")

if __name__ == '__main__':
    socketio.run(
        app,
        host=os.getenv('HOST', '0.0.0.0'),
        port=int(os.getenv('PORT', '5000')),
        debug=os.getenv('DEBUG', 'false').lower() == 'true',
        allow_unsafe_werkzeug=True
    )