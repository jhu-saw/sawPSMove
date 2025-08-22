# sawPSMove

SAW wrapper for PS Move Controllers (PlayStation) based on the
*psmmoveapi* library: https://github.com/thp/psmoveapi. This code
compile on Linux.  This repository provides a core component as well
as:

* Example application with Qt based GUI
* ROS node (also with Qt based GUI)

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * *psmoveapi*: https://github.com/thp/psmoveap
 * Qt for user interface
 * ROS and ROS CRTK (optional) - works with ROS 1 and ROS 2!

# Compilation and configuration

You will first need to download, compile and install the
*psmoveapi*. You can git clone the code from
https://github.com/thp/psmoveap.  Then configure the build with
`ccmake` or `cmake-gui` , set the build type to `Release` and the
install prefix to `/usr/local`.  Build with `make` and install with
`sudo make install`.  

```bash
git clone https://github.com/thp/psmoveapi.git
cd psmoveapi
mkdir build
cd build
```

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
```

```bash
make -j$(nproc)
```

```bash
sudo make install
```

The *psmoveapi* headers, libraries and
executables will be install under `/usr/local`.


You might need to change your BlueTooth configuration, see
https://psmoveapi.readthedocs.io/en/latest/pairing.html (summary, edit
`/etc/bluetooth/input.conf` to set `ClassicBondedOnly=false`).

## Verify Installation

   * Headers:

     ```bash
     ls /usr/local/include/psmoveapi/
     ```

     Expected output: `psmove.h  psmove_config.h  psmove_tracker.h ...`

   * Executables:

     ```bash
     which psmove
     ```
     Expected output: `/usr/local/bin/psmove`

To pair the controllers, since `psmove` should be installed in
`/usr/local/bin`, you can type `psmove pair` and `psmove register`.

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
ros2 run psmove psmove
```

## Other "middleware"

Besides ROS, the ForceDimension component can also stream data to your application using the *sawOpenIGTLink* or *sawSocketStreamer* components.  See:
* [sawOpenIGTLink](https://github.com/jhu-saw/sawOpenIGTLink)
* [sawSocketStreamer](https://github.com/jhu-saw/sawSocketStreamer)
