#!/bin/bash

###################################################################################################

echo "###################################################################################################"
echo "                                  Launch Manual CarROS Script                                      "
echo "###################################################################################################"

cat << EOF
###################################################################################################
If you are running this script on WSL2, you need to configure port forwarding.
Run the following commands on PowerShell (as Administrator):

1. For WebSocket (port 9090):
   netsh interface portproxy add v4tov4 listenaddress=<IP_WINDOWS> listenport=9090 connectaddress=<IP_WSL2> connectport=9090

2. For Web Server (port 8000):
   netsh interface portproxy add v4tov4 listenaddress=<IP_WINDOWS> listenport=8000 connectaddress=<IP_WSL2> connectport=8000

Additionally, ensure the <ip_server> in index.html matches the IP of your Windows machine.
###################################################################################################
EOF

###################################################################################################

# Export ROS_MASTER_URI and ROS_HOSTNAME
export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')

# Print the ROS Master URI and Hostname for confirmation
echo "Exported ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Exported ROS_HOSTNAME: $ROS_HOSTNAME"

###################################################################################################

# Start the Python web server in the correct directory
WEB_DIR=~/catkin_ws/src/ros_car_cmd/web
if [ -d "$WEB_DIR" ]; then
  echo "Starting Python web server in $WEB_DIR..."
  cd "$WEB_DIR"
  python3 -m http.server 8000 &
else
  echo "Directory $WEB_DIR not found. Ensure the web files are correctly located."
  exit 1
fi

###################################################################################################

# ROS environment startup
echo "Launching ROS environment with ros_car_manual.launch..."
roslaunch ros_car_cmd ros_car_manual.launch &

###################################################################################################

IP=$(hostname -I | awk '{print $1}')

cat << EOF
###################################################################################################
The environment is now set up.

1. Open your browser and navigate to:
   http://$IP:8000/index.html

2. Ensure the WebSocket in index.html points to:
   ws://$IP:9090

(if you are using WSL2 remember that the ip must be that of the windows machine)
###################################################################################################
EOF

###################################################################################################
