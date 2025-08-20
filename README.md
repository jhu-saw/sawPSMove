sudo apt-get install -y libusb-1.0-0-dev libbluetooth-dev \
    libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev \
    qtbase5-dev

# Ensure psmoveapi is installed or add its build dir to CMAKE_PREFIX_PATH
# export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$HOME/psmoveapi/build"

# Ensure cisst + cisst_ros_bridge are visible to CMake
# export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/path/to/cisst/install:/path/to/cisst_ros_bridge/install"

mkdir -p ~/ws_psmove/src
cd ~/ws_psmove/src
# drop the 'sawPSMove' folder here
cd ..
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
