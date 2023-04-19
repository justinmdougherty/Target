# Target

At target:
•	LoRa to send and receive messages to/from firing line 
•	GPS for range data
•	Target ID for more than one target expansion
•	Needs to respond to a specific message for calibration that includes GPS data for range
•	Needs to respond when the triggered for hit indicator function

At firing line:
•	Needs to have LoRa to send and receive messages to/from target
•	GPS for range calculation
•	Calibrate function that sends message and calculates RF time of flight to and from target to subtract from shot ToF, also receives GPS position from Target for range calculation and target ID#
•	Displays hit indication and QTY if no firing line trigger (on rifle) and has reset.
•	Displays bullet ToF if rifle trigger and displays running average. Can be reset after any QTY of shots, 10 shots max.
