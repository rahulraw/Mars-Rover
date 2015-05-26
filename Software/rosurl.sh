#!/bin/bash

# rosurl.sh local sets the variables to local ros
# rosurl.sh arthrobot sets variables for remote connectivity
if [ "localhost" = "keshav-N53SV" ]; then
	export ROS_IP="129.97.177.8"
fi

if [ $1 = "arthrobot" ]; then
	export ROS_MASTER_URI=http://<robot-host>:11311 # Set the ROS_MASTER_URI to the fit-pc
	echo "Arthrobot set for remote connection."
elif [ $1 = "local" ]; then
	export ROS_HOSTNAME=localhost			# Set hostname to local
	export ROS_MASTER_URI=http://localhost:11311    # Set ROS_MASTER_URI to local
else
	echo "Incorrect input. Either set to local or arthrobot."
fi


