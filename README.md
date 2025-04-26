# OceanSentinel-Final
Illegal, Unreported, and Unregulated (IUU) fishing threatens marine ecosystems and sustainable fisheries.
This project presents a real-time vessel monitoring solution by integrating EMSOL's Arduino ESP hardware with machine learning algorithms to detect suspicious activities and immediately alert authorities.

Hardware Components
EMSOL Arduino ESP32 (WiFi + Bluetooth enabled)

GPS Module for real-time latitude and longitude acquisition

IoT Sensors for additional environmental/position data capture

Software Components
Python 3.10+ for backend data processing

Machine Learning Models:

K-Means Clustering for identifying movement patterns

Isolation Forest for anomaly detection (suspicious vessel behavior)

Real-Time Alerts:

Email (SMTP Gmail server)

SMS (Twilio API)

Webhook Notifications (for integration into dashboards or authority systems)

Key Features
Real-time Geofencing:

Automatically detects if a vessel crosses Indian Ocean boundaries (20°E–120°E, -60°S–30°N).

Hardware-Software Synergy:

EMSOL Arduino ESP collects GPS data, processed by AI models for prediction and alerting.

Multi-Channel Alerts:

Immediate communication with authorities via Email, SMS, and Webhooks.

Scalable & Cloud-ready:

Easily deployable for large fleet tracking via cloud platforms.

Architecture Diagram
csharp
Copy
Edit
[Arduino ESP32 + GPS Sensor] 
        ↓
[Data Transmission via WiFi]
        ↓
[Python Backend (Geofencing + ML Models)]
        ↓
[Alert Systems: Email | SMS | Webhook]
Setup Instructions
Hardware Setup:

Connect GPS Module to EMSOL Arduino ESP32.

Configure ESP32 to transmit data to server/cloud.

Software Setup:

Install Python dependencies:

bash
Copy
Edit
pip install smtplib twilio scikit-learn requests
Replace credentials for Email, Twilio, and Webhook in config.py.

Run System:

bash
Copy
Edit
python iuu_detection.py
Future Enhancements
Integration with AIS (Automatic Identification System) databases for better vessel authentication.

Mobile app interface for real-time monitoring.

Blockchain ledger for immutable vessel tracking logs.

Novelty and Impact 
Unique hardware-software fusion: Real-time detection by bridging EMSOL Arduino ESP hardware with advanced machine learning.

Fast, Reliable Alerting: Multi-channel notification minimizes reaction time to illegal activities.

Scalable design: Adaptable for national or global marine monitoring networks.

Contributors
Mr. Sarath Kumar (Embedded Systems,EMSOL)
Shobika D
