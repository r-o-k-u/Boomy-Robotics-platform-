import json
from flask import Flask, Blueprint, render_template, request, jsonify
# import numpy as np
# import pandas as pd
# from sklearn.model_selection import train_test_split
# from sklearn.linear_model import LinearRegression
# from sklearn.metrics import mean_squared_error

# app = Flask(__name__)
app = Flask(__name__, template_folder='../templates', static_folder='../static')

# Create a blueprint for the robot control routes
robot_control_blueprint = Blueprint('robot_control', __name__)
app.register_blueprint(robot_control_blueprint)

ip_address = '192.168.0.100'
port_number = 8080
serial_number = 'ABC123'
battery_voltage=12.3
battery_current=2
# Load sensor data from files
def load_sensor_data():
    temperature_file = "./json/temperature_history.json"
    humidity_file = "./json/humidity_history.json"
    pressure_file = "./json/pressure_history.json"

    with open(temperature_file, 'r') as f:
        temperature_data = json.load(f)
    with open(humidity_file, 'r') as f:
        humidity_data = json.load(f)
    with open(pressure_file, 'r') as f:
        pressure_data = json.load(f)

    return {
        "temperature": temperature_data["data"][0],
        "humidity": humidity_data["data"][0],
        "pressure": pressure_data["data"][0]
    }

# Load motor command history from file
def load_motor_command_history():
    with open("./json/motor_command_history.json", 'r') as f:
        motor_commands_data = json.load(f)

    return {
        "forward": motor_commands_data["data"][0],
        "backward": motor_commands_data["data"][1],
        "left": motor_commands_data["data"][2],
        "right": motor_commands_data["data"][3]
    }

# Load frisky transmitter channels data from file
def load_frisky_transmitter_channels():
    with open("./json/frisky_transmitter_channels.json", 'r') as f:
        frisky_transmitter_data = json.load(f)

    return frisky_transmitter_data

# Load battery voltage and current history from files
def load_battery_voltage_history():
    with open("./json/battery_voltage_history.json", 'r') as f:
        battery_voltage_history_data = json.load(f)

    return {
        "labels": battery_voltage_history_data["labels"],
        "data": battery_voltage_history_data["data"]
    }

def load_battery_current_history():
    with open("./json/battery_current_history.json", 'r') as f:
        battery_current_history_data = json.load(f)

    return {
        "labels": battery_current_history_data["labels"],
        "data": battery_current_history_data["data"]
    }

# Route for home page
@app.route('/')
def index():
    sensor_data = load_sensor_data()
    motor_commands_data = load_motor_command_history()
    frisky_transmitter_data = load_frisky_transmitter_channels()
    battery_voltage_history_data = load_battery_voltage_history()
    battery_current_history_data = load_battery_current_history()

    return render_template('index.html', title='Home',battery_voltage=battery_voltage,battery_current=battery_current,ip_address=ip_address,port_number=port_number,serial_number=serial_number, sensor_data=sensor_data, motor_commands=motor_commands_data, frisky_transmitter_channels=frisky_transmitter_data, battery_voltage_history=battery_voltage_history_data, battery_current_history=battery_current_history_data)

# Route for sensor data
@app.route('/sensors')
def get_sensor_data():
    sensor_data = load_sensor_data()
    return render_template('sensor_data.html', title='Sensor Data', sensor_data=sensor_data)

# Route for motor command history
@app.route('/motor_control')
def get_motor_command_history():
    motor_commands_data = load_motor_command_history()
    return render_template('motor_control.html', title='Motor Command History', motor_commands=motor_commands_data)
# Route for sensor data
@app.route('/video_feed')
def get_video_feed():
    return render_template('video_feed.html', title='Video feed')

# Route for frisky transmitter channels data
@app.route('/frisky-transmitter-channels')
def get_frisky_transmitter_channels():
    frisky_transmitter_data = load_frisky_transmitter_channels()
    return render_template('video_feed.html', title='Frisky Transmitter Channels', frisky_transmitter_data=frisky_transmitter_data)

@app.route('/robot_status')
def get_robot_status():
    # Get the current status of the robot from the database or other source
    # For now, just return a hardcoded value
    status = 'Running'
    return jsonify({'status': status})

@app.route('/robot_control', methods=['POST'])
def control_robot():
    data = request.get_json()
    # Use the data to control the robot (e.g., send commands to the motor drivers)
    # For now, just simulate the command
    print(data)
    return jsonify({'message': 'Robot commanded successfully'})

# Create a blueprint for the sensor data routes
sensor_data_blueprint = Blueprint('sensor_data', __name__)
app.register_blueprint(sensor_data_blueprint)



# Create a blueprint for the motor control routes
motor_control_blueprint = Blueprint('motor_control', __name__)
app.register_blueprint(motor_control_blueprint)

@app.route('/motors/<int:motor_id>', methods=['POST'])
def control_motor(motor_id):
    data = request.get_json()
    # Use the data to control the specified motor (e.g., send commands to the motor driver)
    # For now, just simulate the command
    print(data)
    return jsonify({'message': f'Motor {motor_id} commanded successfully'})

if __name__ == '__main__':
    app.run(debug=True, port=5001)
