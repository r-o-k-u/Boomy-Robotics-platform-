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