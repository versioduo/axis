# V2 axis

🧭 Orientation Sensor Controller

Sends MIDI Control Change messages with Quaternion or Euler orientation data.

The device can be mounted at any orientation or angle, and calibrated to align its axes
and set to zero position:

-   The device, or the body it it mounted on, points upwards / to the sky. A double-click
    of the the button. The LED turns cyan.

-   The device points forward, a single click. The LED turns orange. The axes of the sensor
    are aligned to world coordinates and the position reset to zero.

-   Any single click will reset the position to zero, but not change the calibration.

-   A long-press double-click will save the calibration to the EEPROM. The LED flashes purple.

This sequence identifies the x axis (the axis of rotation between the button presses)
and the negative z axis (the gravity at the second step).

![axis hardware](images/front.jpg?raw=true)

## Hardware

![Schematics / PCB Repository](https://github.com/versioduo/axis-hardware)

## Visualization

[versioduo.com/axis3d](https://versioduo.com/axis3d/)

![Screenshot](images/axis3d.png?raw=true)
