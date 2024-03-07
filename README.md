# Saros_Balloon_Computers
Code for the balloons launching into the eclipse as part of Project Saros. 


Two possible configurations exist:
1. Heater code is loaded on a separate computer
   A. One computer logs sensors, the other is dedicated for controlling heaters using sensors
   B. This makes logging data from the operational temperatures more difficult
2. The main flight computer operates both data logging and control of heaters and sensors
   A. This makes one computer for everything
   B. Pin counts could be a limitation
   C. Software crashing could lead to heaters failing

Proposing option 2. One computer

Software general layout: 
Main loop operating these functions:
1. Data log (1Hz?)
2. Gyroscope/Accelerometer log (fastest rate possible?)
3. Heat Control (0.1Hz?)
4. Optional payloads

Known data log:
1. GPS
2. Pressure/altitude
3. Experimental temperature (various locations)
4. Operational temperature (paired with heaters)
5. Inertial Measurement Unit (IMU)
   A. 3-axis Gyroscope
   B. 3-axis accelerometer
   C. 3-axis magnetometer



Optional payloads could include:
Additional GPS units
Satellite trackers
Anything anyone decided to add!
