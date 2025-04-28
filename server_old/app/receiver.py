# RPi (install flask-socketio)
from flask import Flask, render_template
from flask_socketio import SocketIO
import serial

app = Flask(__name__)
socketio = SocketIO(app)
ser = serial.Serial('/dev/ttyACM0', 115200)

@app.route('/')
def index():
    return render_template('control.html')

@socketio.on('connect')
def handle_connect():
    while True:
        line = ser.readline().decode().strip()
        socketio.emit('update', {'data': line})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)