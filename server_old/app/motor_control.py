from flask import render_template

@app.route('/motor_control')
def motor_control():
    # Simulate motor control data
    motor_commands = ['forward', 'backward', 'left', 'right']
    return render_template('motor_control.html', motor_commands=motor_commands)
