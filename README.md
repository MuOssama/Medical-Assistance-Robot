# Medical Assistance Robot

This project is a comprehensive medical assistance robot developed using the Robot Operating System (ROS). It integrates various functionalities to support patient care, including health monitoring, data management, mobility, medication dispensing, and emergency response.

## Repository Structure

- **Despenser/**: Manages medication dispensing, handling both syrups and tablets.
- **emergencyNode/**: Monitors patient health metrics to detect emergencies and initiate alerts.
- **motion/**: Controls the robot's movement, enabling navigation within healthcare environments.
- **smartMedicalWatch/**: Interfaces with wearable devices to collect real-time patient health data.
- **static/**: Contains configuration files and resources essential for the robot's operation.
- **patients.db**: A database file that stores patient information securely.
- **ros cheat notes.txt**: A reference file with ROS commands and notes for development purposes.
- **server.py**: Implements the server responsible for managing patient data and coordinating between nodes.

## Key Features

- **Health Monitoring**: The smart medical watch node collects vital signs and health metrics from patients in real-time.
- **Data Management**: The server node maintains patient records, ensuring data is up-to-date and accessible.
- **Mobility**: The motion node enables the robot to navigate autonomously, assisting patients as needed.
- **Medication Dispensing**: The dispenser node administers medications accurately, managing both syrups and tablets.
- **Emergency Response**: The emergency node detects critical health issues and alerts medical personnel promptly.


