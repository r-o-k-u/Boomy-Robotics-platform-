from flask import render_template
import matplotlib.pyplot as plt

@app.route('/sensor_data')
def sensor_data():
    # Simulate sensor data
    x = [1, 2, 3]
    y = [4, 5, 6]
    return render_template('sensor_data.html', x=x, y=y)
