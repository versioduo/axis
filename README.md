# V2 axis
🧭 MIDI Orientation Sensor and Accelerometer

Sends MIDI Control Change messages with Quaternion or Euler orientation data.

![axis hardware](images/front.jpg?raw=true)

## Setup
The device can be mounted at any orientation or angle, the reference frame / axes can be aligned to the current posion and stored in the device. This sequence identifies the Y axis (the axis of rotation between the button clicks) and the Z axis (the gravity at the second step):

- The device, or the body it is mounted on, points upwards / to the sky. A double-click of the the button, the LED turns green.

- The device points forward, the zero position. A single click, the LED turns orange. The axes of the sensor are now aligned and the rotation reset to zero.

- Any single click will set the rotation to zero, but not change the reference frame.

- A long-press double-click will store the calibration in the device, the LED flashes purple.

A long-press triple-click erases the stored data, the LED flashes purple.

## Control Change Mapping
Holding the device button down, suppresses all Control Change messages. The CC numbers can be configured in the Setting section. Pressing the blue button there, sends this single CC message; it can be used to map the CC in the Audio Workstation."

## Visualization
[versioduo.com/axis3d](https://versioduo.com/axis3d/)

![Screenshot](images/axis3d.png?raw=true)

# Copying
Anyone can use this public domain work without having to seek authorisation, no one can ever own it.
