# Autonomous Surveillance Robot

Raspbian for robots by Dexter Industries version 2.4.4

This code calculate the optimal path for a hardcoded given map (fixed obstacles) using A* algorithm. The path finding output is used to calculate the direction and move the Gopigo3/Raspberry pi 3 Robot. The ultrasonic sensor is responsable to detect moving objects and alert the operator.

In order to use the project, just clone it and execute asr.py file using python3.

python3 asr.py

If using Raspbian for robots by Dexter Industries operating system and the Raspberry Pi camera module v2, do the following to start the camera web server and view the user interface

1. Connected to the Raspbian terminal, browser to the directory:

cd Desktop/GoPiGo/Software/Python/Examples/Browser Streaming Robot/

2. Make the robot_web_server program as executable:

sudo chmod +x robot_web_server.py

3. Start the robot_web_server.py to start streaming the video from the camera module:

sudo ./robot_web_server.py

4. Now open the browser and enter dex.local:98 in the browser and hit enter to start streaming. You’ll also see some activity in the terminal.
