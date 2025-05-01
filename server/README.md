# Create virtual enviroment
```
python -m venv venv
```


# On Windows
```
venv/Scripts/activate
```

# On macOS/Linux
```
source venv/bin/activate
```

# To run

# To use these:

# For development machines (Windows/Linux):

```

 pip install -r requirements-dev.txt


 pip install -r requirements-wi.txt

```
## For Raspberry Pi:

```
pip install -r requirements-rpi.txt

```

# to run
```
python app/__init__.py --port=5001

```
###

Installation Requirements:

bash
# Linux
sudo apt-get install libfreenect-dev
pip install freenect

# Windows
pip install freenect
# Install WinUSB driver using Zadig for "Xbox NUI Motor"
Here's how to install the WinUSB driver for Kinect v1 using Zadig:

Step-by-Step Guide
Download Zadig
Get the latest version from the official site:
https://zadig.akeo.ie

Connect Kinect v1
Plug your Kinect into a USB 2.0+ port and wait for Windows to detect it.

Run Zadig as Administrator
Right-click Zadig → "Run as administrator"

Configure Zadig:
Go to Options → List All Devices

Zadig Options

Select Device:
From the dropdown, choose "Xbox NUI Motor (Interface 0)"

Select Xbox NUI Motor

Install WinUSB Driver:

Driver: Select WinUSB from the right-side dropdown

Click "Replace Driver"

Install WinUSB

Wait for Completion
You'll see a success message when done:
Driver Installation Complete!

Verification
Check Device Manager:
Under "Universal Serial Bus devices", you should see:
"WinUSB Device"

Device Manager

Test with Python:

python
import freenect
print(f"Found {freenect.num_devices()} Kinect devices")
Test with CLI Tool:

bash
freenect-glview
Troubleshooting
Issue	Solution
Device not listed	Unplug/replug Kinect, ensure USB 2.0+ port
Driver install fails	Disable driver signature enforcement (Windows Advanced Startup)
Access denied errors	Run Zadig as Administrator
Depth sensor not working	Repeat steps for "Xbox NUI Motor (Interface 1)"
Important Notes
Do this once per USB port you use with Kinect

Required for both Windows 10 and 11

Not needed for Linux (works out-of-box with libfreenect)

After completing these steps, your Kinect v1 will work with libfreenect in Python! 🎉

pip install flask numpy opencv-python
sudo apt-get install libfreenect-dev  # For Linux


pip freeze > requirements.txt
# Or use homebrew on macOS: brew install libfreenect

# Install Python bindings
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect/wrappers/python
python setup.py install


Key Dependencies Explained:

Web Framework:

flask: Core web server

flask-socketio: Real-time WebSocket communication

eventlet: Async server for SocketIO

Hardware Control:

pyserial: Serial communication with Arduino

opencv-python-headless: Camera processing

freenect/pykinect2: Kinect support

Mapping:

geopy: GPS coordinate calculations

gpxpy: Route planning and tracking

Optional:

RPi.GPIO: Raspberry Pi hardware control

adafruit-circuitpython-servokit: Motor control

Development Recommendations:

Create separate requirement files:

requirements.txt (core)

requirements-win.txt (Windows-specific)

requirements-pi.txt (Raspberry Pi)

For production deployment:

text
Copy
# requirements-prod.txt
flask==2.0.1
flask-socketio==5.1.1
pyserial==3.5
opencv-python-headless==4.5.5.64
gunicorn==20.1.0  # Production server


Key Features & Enhancements
Complete Hardware Integration

Kinect v1 RGB and Depth streams

Webcam fallback with automatic detection

Serial communication with retry logic

Motor control commands

Kinect LED and tilt control

Enhanced Error Handling

Graceful hardware failure recovery

Automatic webcam fallback

Serial connection retries

Comprehensive error logging

Real-Time Data System

WebSocket telemetry updates

GPS position tracking

Multi-threaded data processing

Asynchronous video streaming

Security Features

HTTP Basic Authentication

Configurable credentials

Secure secret management

Input validation

Documentation & Maintainability

Full PEP8 compliance

Detailed docstrings

Type hinting

Configuration via environment