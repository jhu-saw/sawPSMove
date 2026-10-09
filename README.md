# sawPSMove

SAW wrapper for PS Move Controllers (PlayStation) based on the
*PSMoveAPI* library: https://github.com/thp/psmoveapi. This code
compiles on Linux.  This repository provides a core component as well
as:

* Example application with Qt based GUI
* ROS node (also with Qt based GUI)

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * PSMoveAPI build dependencies (the source is downloaded automatically)
 * Qt for user interface
 * ROS and ROS CRTK (optional) - works with ROS 1 and ROS 2!

# Compilation and configuration

By default, sawPSMove downloads a pinned PSMoveAPI revision and builds
the core library, tracker library and `psmove` command-line utility.
The build doesn't install anything system-wide and doesn't require a
pre-existing PSMoveAPI installation.

```bash
sudo apt install build-essential cmake git libbluetooth-dev libdbus-1-dev \
  libopencv-dev libudev-dev libusb-1.0-0-dev libv4l-dev
```

The bundled revision can be changed without editing the CMake files:

```bash
colcon build --packages-select sawPSMoveCore \
  --cmake-args -DSAW_PSMOVEAPI_GIT_TAG=<commit-or-tag> \
               -DSAW_PSMOVEAPI_UPDATE=ON
```

For development, use a commit hash so builds remain reproducible.  The
default is the upstream commit tested with sawPSMove.  Remote updates
are disabled after the initial clone unless `SAW_PSMOVEAPI_UPDATE` is
enabled.

To use an existing PSMoveAPI installation instead, select the system
provider:

```bash
colcon build --packages-select sawPSMoveCore \
  --cmake-args -DSAW_PSMOVEAPI_PROVIDER=SYSTEM
```

You might need to change your Bluetooth configuration; see
https://psmoveapi.readthedocs.io/en/latest/pairing.html.  Use `psmove
pair` and `psmove register` to pair controllers.

# Build

See https://github.com/jhu-saw/vcs for download and build instructions.  Use the VCS files for `ps-move`.


# Examples

## Main example

The main example provided is `sawPSMoveQtExample`.  The command line options are:
```sh
sawPSMoveQtExample:
 -j <value>, --json-config <value> : json configuration file (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

## ROS

If you also want to use the ROS node for ROS 1, run:
```sh
rosrun ps_move ps_move
```

For ROS 2, run:
```sh
ros2 run ps_move ps_move
```

## Other "middleware"

Besides ROS, the PSMove component can also stream data to your
application using the *sawOpenIGTLink* or *sawSocketStreamer*
components. See:
* [sawOpenIGTLink](https://github.com/jhu-saw/sawOpenIGTLink)
* [sawSocketStreamer](https://github.com/jhu-saw/sawSocketStreamer)
