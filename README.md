# Automated Pill Dispenser for Medication Adherence

## Project Overview
This embedded systems project addresses medication adherence challenges among elderly patients, where non-compliance rates range from 40-75%. The automated pill dispenser ensures timely medication delivery through a programmable, reliable system that provides visual and auditory reminders.

## Awards
🏆 **First Place Winner** - CMPE443 Embedded Systems Design Competition (2024-2025)

## Demo
[View System Demonstration Video](https://drive.google.com/file/d/1RxHljePmvkYLW-22ZvhPf_oIgyRR5TJg/view?usp=sharing)

## Key Features
- **Programmable Timing System**: Set precise medication dispensing schedules through computer interface
- **Countdown Display**: 7-segment display shows time remaining until next dose
- **Automated Dispensing**: Servo motor rotates to dispense pills at scheduled intervals
- **Presence Detection**: Ultrasonic sensor detects when medication has been dispensed
- **Multi-modal Alerts**: LED and buzzer notifications with adjustable timing intervals
- **User Controls**: Start/reset buttons and potentiometer for alert customization
- **Bluetooth Connectivity**: Wireless communication between system components
- **Logging System**: Records dispensing events for monitoring adherence

## Technical Implementation
- Full interrupt-driven architecture for efficient operation
- STM32 microcontroller-based system with dual board design (master/slave)
- UART/Bluetooth communication protocol
- Power-efficient design with battery operation capability
- Ultrasonic sensing for pill detection
- Timer-based control systems

## Components
- STM32L552ZE-Q microcontrollers (master and slave)
- HC-05 Bluetooth modules
- HC-SR04 ultrasonic sensor
- HS-311 servo motor
- 3-digit seven-segment display
- LED, buzzer, and input controls

## Physical Design & Sanitation Features

- **Individual Compartmentalized Boxes**: Medications are stored in separate, small containers rather than loose pills, preventing cross-contamination
- **Easy-Load Design**: Removable pill boxes can be filled and organized by caregivers or pharmacists in advance
- **Rotation Mechanism**: Internal carousel system precisely aligns and dispenses the correct pill box at scheduled times
- **Sterile Materials**: Pill containers made from medical-grade, non-porous materials that resist bacterial growth
- **Cleanable Surfaces**: All contact surfaces designed with smooth, easily cleanable finishes
- **Hygienic Dispensing**: Touchless delivery system minimizes contamination risks during medication retrieval
- **Sealed Storage**: Internal storage area protects medications from environmental exposure and contaminants
- **Compact Design**: Each compartment optimized for storage efficiency while accommodating various pill sizes
- **Adjustable Audio-Visual Alerts**: System uses buzzer and LED to notify patients when medication is dispensed, with customizable beep intervals controlled via potentiometer (0.5-5 seconds)
- **Ergonomic Retrieval**: Easy-access dispensing area designed for users with limited dexterity
- **Compliance Tracking**: System records when pill boxes are removed, creating an adherence history
- **Bluetooth-Based Logging**: Medication dispensing and retrieval events transmitted via Bluetooth to local control system, with architecture designed for easy integration with remote monitoring solutions
- **Real-Time Status Dashboard**: Web interface displays current medication schedule and adherence history

This project demonstrates embedded systems integration, real-time programming, sensor integration, and human-centered design for healthcare applications.
