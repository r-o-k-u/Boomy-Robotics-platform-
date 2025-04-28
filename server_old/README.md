# Create virtual enviroment
```
python -m venv venv
```


# On Windows
```
venv\Scripts\activate

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
# if installs fail 
python -m venv --system-site-packages venv
source venv/bin/activate
python -m pip install -r requirements-rpi.txt
# to run
```
python app/__init__.py --port=5001

```
###

pip install flask numpy opencv-python
sudo apt-get install libfreenect-dev  # For Linux
# Or use homebrew on macOS: brew install libfreenect

# Install Python bindings
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect/wrappers/python
python setup.py install


# 1. Hardware Connections
Radio Receiver (IBus compatible) to Raspberry Pi:

Receiver       Raspberry Pi
---------------------------
VCC        ->  5V
GND        ->  GND
Signal     ->  GPIO15 (UART RX)
Arduino Mega to Raspberry Pi:

Copy
Arduino Mega   Raspberry Pi
---------------------------
USB           -> USB Port
# 2. Raspberry Pi Configuration
Enable Serial Interface:

sudo raspi-config
# -> Interface Options -> Serial -> Disable console, Enable hardware
sudo reboot


#  System Diagram
Radio Receiver -> Raspberry Pi (UART) -> IBus Processing
               \
Arduino Mega <- Raspberry Pi (USB) -> Motor Control
# Verification Steps
# Check serial devices:

ls /dev/tty*
# Should see /dev/ttyS0 (receiver) and /dev/ttyACM0 (Arduino)

# Test receiver input:

print(rover.receiver.read())
# Test motor control:

rover.set_motor_speeds(1500, 1500)  # Neutral position
#  Final System Configuration
Add these lines to /boot/config.txt:


enable_uart=1
dtoverlay=disable-bt