import roboclaw

rb = roboclaw.RoboClaw('/dev/ttyACM0')

rb.M1Forward(100);