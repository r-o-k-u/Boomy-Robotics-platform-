import json
import platform
import random
import time
from datetime import datetime
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
import cv2
import threading


class IBus:
    def __init__(self, uart_num=0, baud=115200, num_channels=10):
        self.num_channels = num_channels
        self.ch = [0] * (self.num_channels + 1)  # ch[0] is status

        if 'raspberrypi' in platform.uname().release.lower():
            try:
                try:
                    from machine import UART
                except ImportError:
                    UART = None  # Handle the absence of the machine module
            except ImportError:
                UART = None  # Handle the absence of the machine module
            self.uart = UART(uart_num, baud)
        else:
            self.uart = None

    def read(self):
        if self.uart:
            # Actual hardware reading implementation
            for _ in range(10):  # Max 10 attempts
                if self.uart.read(1) == b'\x20':  # Start byte found
                    buffer = self.uart.read(23)
                    if buffer and len(buffer) == 23:
                        checksum = 0xFFFF - 0x20
                        for b in buffer[:21]:
                            checksum -= b
                        received_checksum = (buffer[22] << 8) | buffer[21]
                        if checksum == received_checksum:
                            self.ch[0] = 1
                            for i in range(1, self.num_channels + 1):
                                self.ch[i] = buffer[(i-1)*2] | (buffer[(i-1)*2 + 1] << 8)
                            return self.ch
        else:
            # Fallback to simulated data if not on Raspberry Pi
            return IBusSimulator().read()
        return [0] * (self.num_channels + 1)

class IBusSimulator:
    def __init__(self):
        self.ch = [1] + [random.randint(1000, 2000) for _ in range(10)]

    def read(self):
        # Simulate changing values with proper value clamping
        self.ch = [1] + [
            max(min(c + random.randint(-10, 10), 2000), 1000)
            for c in self.ch[1:]
        ]
        return self.ch
# ================= Hardware Abstraction Layer =================
class RoverCore:
    def __init__(self):
        self.is_raspberry = self._check_raspberry()
        self.control_source = 'transmitter'
        self._init_hardware()
        self._init_sensors()
        self.kinect = None
        self._init_kinect()

    def _check_raspberry(self):
        return 'raspberrypi' in platform.uname().release.lower()

    def _init_hardware(self):
        if self.is_raspberry:
            self._init_pi_hardware()
        else:
            self._init_simulated_hardware()

    def _init_pi_hardware(self):
        import RPi.GPIO as GPIO
        from imusensor.MPU9250 import MPU9250

        # Motor Control Setup
        GPIO.setmode(GPIO.BCM)
        self.motors = {
            'left': GPIO.PWM(17, 1000),  # Front-Left Motor
            'right': GPIO.PWM(18, 1000)  # Front-Right Motor
        }
        for motor in self.motors.values():
            motor.start(0)

        # IMU Setup
        self.imu = MPU9250.MPU9250()
        self.imu.begin()

        # Receiver Setup
        self.receiver = IBus(uart_num=0)

    def _init_simulated_hardware(self):
        self.receiver = IBusSimulator()
        self.imu = None

    def _init_sensors(self):
        self.sensor_data = {
            'battery': {'voltage': 12.6, 'current': 2.1, 'capacity': 5.0},
            'imu': {'accel': [0]*3, 'gyro': [0]*3, 'mag': [0]*3},
            'encoders': {'left': 0, 'right': 0},
            'channels': [1500]*10
        }

    def _init_kinect(self):
        if self.is_raspberry:
            self.kinect = cv2.VideoCapture(0)
        else:
            self.kinect = cv2.VideoCapture(0)  # Use webcam for simulation

    def get_kinect_frame(self):
        if self.kinect and self.kinect.isOpened():
            ret, frame = self.kinect.read()
            if ret:
                return cv2.imencode('.jpg', frame)[1].tobytes()
        return None

    def update_sensors(self):
        if self.is_raspberry:
            self._update_pi_sensors()
        else:
            self._simulate_sensors()

    def _update_pi_sensors(self):
        # Read actual hardware sensors
        self.sensor_data['imu']['accel'] = self.imu.get_accel()
        self.sensor_data['imu']['gyro'] = self.imu.get_gyro()
        self.sensor_data['channels'] = self.receiver.read()[1:]

        # Simulate encoder readings
        self.sensor_data['encoders']['left'] = int(random.uniform(0, 1000))
        self.sensor_data['encoders']['right'] = int(random.uniform(0, 1000))

    def _simulate_sensors(self):
        # Generate realistic simulated data
        self.sensor_data['imu']['accel'] = [random.gauss(0, 0.1) for _ in range(3)]
        self.sensor_data['imu']['gyro'] = [random.gauss(0, 0.5) for _ in range(3)]
        self.sensor_data['encoders'] = {
            'left': int(random.uniform(0, 1000)),
            'right': int(random.uniform(0, 1000))
        }
        self.sensor_data['channels'] = self.receiver.read()[1:]

    def set_motor_speeds(self, left, right):
        if self.is_raspberry and self.control_source == 'ui':
            self.motors['left'].ChangeDutyCycle(left)
            self.motors['right'].ChangeDutyCycle(right)

# ================= Flask Application Setup =================

app = Flask(__name__, template_folder='../templates', static_folder='../static')
socketio = SocketIO(app, async_mode='threading')
rover = RoverCore()
# Initialize camera
camera = cv2.VideoCapture(0)

# ================= Video Streaming Endpoint =================
def gen_frames():
    while True:
        frame = rover.get_kinect_frame()  # Attempt to get Kinect frame
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Kinect not connected or failed to get frame, fallback to default camera
            success, fallback_frame = camera.read()
            if success:
                _, buffer = cv2.imencode('.jpg', fallback_frame)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ================= WebSocket Handlers =================
@socketio.on('connect')
def handle_connect():
    threading.Thread(target=sensor_update_loop).start()

def sensor_update_loop():
    while True:
        rover.update_sensors()
        # print({
        #     'telemetry': rover.sensor_data,
        #     'channels': rover.sensor_data['channels'],
        #     'battery': rover.sensor_data['battery']
        # })
        socketio.emit('sensor_update', {
            'telemetry': rover.sensor_data,
            'channels': rover.sensor_data['channels'],
            'battery': rover.sensor_data['battery']
        })
        time.sleep(0.1)

# ================= Control Endpoints =================
@app.route('/api/control/motors', methods=['POST'])
def control_motors():
    if rover.control_source == 'ui':
        data = request.json
        rover.set_motor_speeds(data['left'], data['right'])
        return jsonify(success=True)
    return jsonify(success=False, error="Transmitter control active")

@app.route('/api/control/source', methods=['POST'])
def set_control_source():
    data = request.json
    rover.control_source = data['source']
    return jsonify(success=True)

# ================= Dashboard Routes =================
@app.route('/')
def control_dashboard():
    return render_template('dashboard.html')

# ================= Main Execution =================
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
