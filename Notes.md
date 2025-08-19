# What to keep in mind

## 0) Start from an existing SAW device repo and rename

* Pick a simple SAW/cisst “device” package (e.g., OptoForce or SensAble/Phantom example he showed).
* Copy it, **strip what you don’t need**, and **rename everything** (files, classes, CMake targets, `package.xml`) from “OptoForce/HD/etc.” → **PSMove**.
* Keep the standard scaffolding: `core/` (components + example), `ros/` (ROS/CRTK node, optional), minimal `share/` (likely empty for now).

## 1) Use a single mtsTaskContinuous

* This component is basically “a while loop wrapped in cisst.”
* Use **`mtsTaskContinuous`**. That’s the right place to poll the PSMove at a fixed period and publish pose.
* Keep **Component Manager**, **Interval statistics**, **message logger**, and the **dark mode** option as-is—zero reinvention there.

## 2) Minimal provided interface (CRTK-friendly)

* **State table member:** `M_measured_cp` (measured Cartesian pose: position + rotation).

  * Type: `vctFrm3` (or `prmPositionCartesianGet` to carry timestamp/valid flags cleanly).
* **Read command(s):**

  * `measured_cp` (that’s enough to start).
  * Later: buttons, “valid” flag, jaw angle, etc.
* **State machine commands (standard CRTK mapping):**

  * `enable()`, `disable()`, `home()` (map to connect, disconnect, calibrate).
  * Read backs: `is_enabled`, `is_homed`, maybe `operating_state`.
  * Early on you can **stub** these to “enabled & homed” to unblock testing; tighten later by checking real PSMove connection/cal status.

## 3) Run loop = poll → convert → publish

* Each cycle:

  1. Poll PSMove (quat + 3D position surrogate if you have one; orientation for sure).
  2. Convert **quaternion → rotation matrix** with cisst utilities.
  3. Fill `M_measured_cp` + timestamp + **valid**.
  4. Push to state table so CRTK Bridge auto-exposes `measured_cp`.
* If API reports “not tracking / no orientation / low battery,” set **valid = false**.

## 4) Keep GUI lightweight (optional, but handy)

* Reuse cisst Qt widgets:

  * **3D pose widget** (for `measured_cp`),
  * **Interval statistics widget** (loop timing),
  * **Message widget** (logs).
* You do **not** need a custom widget; hook the standard ones to your interface. It’s a few lines.

## 5) ROS1 vs ROS2, CRTK bridge

* The **existing main(s)** already handle ROS1/ROS2 differences & CRTK wrapping. Keep that pattern.
* Your job is just to **create the device component** and expose CRTK-ish names; the bridge will “see” `measured_cp` and publish it properly.

## 6) Multi-controller support (future-proof now)

* Support **2 controllers** out of the box (Right, Left).
* Naming: first discovered = Right, second = Left (simple convention, no JSON needed).
* Structure the component so you can instantiate **two devices** easily (or one device managing N PSMove handles).

## 7) Configuration: keep it minimal

* Skip JSON/config at first. Hardcode: discover first (and second) controller.
* Later: optional names/IDs/calibration via JSON once basic path works.

## 8) Calibration/home mapping

* Map **`home()`** → “ensure orientation calibration complete” (PSMove magnetometer/IMU calibration flow).
* Map **`is_homed`** to “has orientation + calibration OK”.
* `enable()` can attempt BT connect if not connected.

## 9) Cross-platform guards

* Keep the **Windows/Mac export headers**; PSMove works on Linux/macOS/Windows, so don’t drop those.

## 10) Testing strategy

* First build the **core example app** (no ROS) that creates your device and shows the 3 widgets; move controller around and watch pose update.
* Then build the **ros** target; verify `/measured_cp` via CRTK Bridge.
* Add a “valid” indicator in the widget to quickly spot tracking loss.

---

# Minimal skeleton (what you actually implement)

**Files to keep/adapt**

* `core/components/`

  * `sawPSMoveController.h/.cpp` (the component)
  * `export.h` (platform exports)
* `core/examples/`

  * `sawPSMoveExampleQt.cpp` (no-ROS Qt test app)
* `ros/src/`

  * `saw_psmove_ros.cpp` (ROS/CRTK main patterned after OptoForce/Phantom)
* Build/package:

  * `CMakeLists.txt`, `package.xml` (rename targets, deps: cisst, sawCommon, Qt5/6, PSMove)
  * Optional: `concurrentpackage.xml` / any template Anton mentioned—just **rename occurrences** (OptoForce→PSMove).
