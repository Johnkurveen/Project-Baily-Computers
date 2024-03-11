# Project Baily Computers
Code for the balloons launching into the eclipse as part of Project Baily. Name may change


Heater code is loaded on a separate computer
   A. One computer logs sensors, the other is dedicated to controlling heaters using sensors
   B. Sensors for the heat control run wires to both computers. One logs the data, the other uses it for control without logging.
   C. The purpose is to make sure a software bug in the data logger doesn't disable heating 
   
This heat computer may be removed at a later date but active heating seems likely at this time. 
   


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
