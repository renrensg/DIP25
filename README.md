# DIP25-
NTU Project DIP 2025

Drone with Static Camera AI-IoT 

🧑‍💻 Software Team
(Karun | Jarren W.)

The software team engineered the full software pipeline powering the stagnant-water detection and reporting system. Their work brings together computer vision, GPS streaming, and wireless communication to create a reliable real-time detection platform on the Raspberry Pi 4.

✅ AI Model Training completed on Roboflow using the RF-DETR (Nano) architecture.
✅ All functional testing and development logs are maintained in GitHub.
✅ Main focus: AI-Camera pipeline + Rover coordination for real-world outdoor detection.

What We Worked On:

🧪 Functional Testing (GitHub)

All major feature tests, logs and development code are documented in the repo’s /testing and /modules folders.

🚗 Rover & AI-Camera Coordination

Designed communication between the Raspberry Pi and the rover’s Arduino-based control system.
Built wireless transmission using HC-12 UART to sync detection events with rover movement.
Ensured the AI camera module reliably reports detection, GPS, and status data.

🛠️ Development & Testing Journey
🔧 Motor & Rover Tests

Tested individual DC motors for direction, PWM speed control, and stall behaviour.
Validated the motor driver wiring, power isolation, and emergency stop behaviour.
Refined rover movement to keep motion smooth even under low battery voltage.

💨 Pump & Relay Validation

Controlled the pump using an Arduino relay test sketch.
Verified suction strength, placement on the chassis, and timing control.
Debugged relay noise issues and researched lower-voltage relay alternatives when interference occurred.

🍓 Raspberry Pi Integration

Installed and configured OpenCV, PyTorch, PySerial, pynmea2, and YOLO dependencies.
Optimised model loading, frame preprocessing, and numpy operations for faster inference.
Established a stable VNC wireless link so the rover and AI camera could be monitored remotely.

📡 UART Communication (GPS & HC-12)

Built serial communication for the GPS transceiver using PySerial.
Parsed NMEA sentences (like GPGGA) through pynmea2 to extract live coordinate data.
Set up UART with HC-12 for long-range bidirectional data: commands, detections, and status signals.

🧩 Full System Test to Prototype

Once all modules were stable, we stitched everything together:
AI detection triggers → GPS tagging → rover command → data transmission.
Live camera inference ran while GPS streamed coordinates in the background.
The system was stress-tested in different lighting and rover-driving conditions.
Completed end-to-end demo showing real-time stagnant-water detection feeding into rover response.

✅ Conclusion

The software team successfully built a complete AI-IoT pipeline, from training the model (on Roboflow) to integrating every hardware and communication module. By combining computer vision, GPS tracking, wireless UART communication and rover coordination, we delivered a fully functional prototype ready for field demonstration. The system is lightweight, reliable, and optimised for real-time deployment, which is a strong foundation for future expansion into our drone-based or multi-node monitoring networks.

Stay tune for more updates! Nov 2025



