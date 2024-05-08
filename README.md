# APPT-offline-controller
Offline CNC fob for FluidNC/grbl APPT router
---
Presents a easy(/ier) to use UI for a CNC router performing APPT treatment of planar composite material.

Hardware used:
- CNC router
    1. MKS TinyBee
    2. 3x TB6600
    3. MAX485 breakout
- Handheld fob
    1. Generic RepRapDiscount 20x4 Smart Controller
    2. Arduino Micro
    3. MAX485 breakout

todo:
- tune steps per mm, accelerations, max feedrates
- install and configure endstops
- construct, test and debug CNC fob (don't even know if any of the stuff works yet)