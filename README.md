# ODT-quadruped-robot
3D model files, code, circuit, components of a robot dog
This project documents the process of designing, building, and testing a small quadruped robot. It did not succeed, but it is a documentation nonetheless. 

📚 Overview
	•	Goal: Build a functional four-legged walking robot using an ESP32, servo motors, and custom-designed mechanical parts.
	•	Challenges: Stability, joint design, gait, etc.
	•	Outcome: A physical prototype that could stand, move slightly, but needs major improvements in stability, weight distribution, and mechanical joints.

⸻

🛠 Hardware
	•	ESP32 microcontroller
	•	PCA9685 servo driver (16-channel PWM board)
	•	8x Tower Pro SG90 servos
	•	Ultrasonic sensor (HC-SR04 for basic obstacle detection)
	• 12V Adapter with MB102 Power Supply Module 
	•	Custom 3D printed body (PLA)
	•	Miscellaneous: Perfboard, jumper wires, feviquik, masking tape, etc.

⸻

💻 Software
	•	Programming Language: MicroPython
 
  •	Libraries Used:
	•	machine, time (MicroPython standard)
	•	Custom I2C code for PCA9685 communication (no external libraries used)
 
  •	Development Tools:
	•	Mu Editor
	•	Fusion 360 (CAD modeling)
	•	Vizcom (concept sketches)
