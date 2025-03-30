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
