# Turtlebot_Navigation
This repository contains ROS packages for a turtlebot simulation that navigates to goal location using different behaviours such as - follow wall, avoid obstacles, go to goal. The package doesn't use the ROS Navigation Stack but uses a series of logic expressions to achieve the desired aim leveraging the above mentioned behaviours. 

Result: The turtlebot is successfully able to go to the desired position in space without colliding with any obstacles. 

To Simulate it yourself:

1. Clone the repository in 'src' folder of your workspace: git clone https://github.com/jagani-aditya/Turtlebot_Navigation.git
2. Go back to <your_ros_workspace> using 'cd ..' 
3. catkin_make
4. source devel/setup.bash
5. roslaunch robot_simulation robot_gazebo.launch


You should see gazebo getting launched. The robot shall by default spawn at x: -1 y:-2 and navigate to goal position: x: 10 y: 10
In order to change the goal positions ->  cd <your_ros_workspace>/src/Turtlebot_Navigation/robot_simulation/scripts
Open navigate_vector.py and change the 'x_goal' 'y_goal' values in the def __init__(self) function



Troubeshoot Issues: ERROR: cannot launch node of type [robot_simulation/navigate_vector]: can't locate node [navigate_vector] in package [robot_simulation]
To solve the above issue, simply go to the robot_simulation folder -> go to scripts folder -> In terminal type: chmod +x navigate_vector.py
Now that you've made your file executable enter the following command in terminal: roslaunch robot_simulation robot_gazebo.launch
The gazebo will launch and the robot shall automatically start moving. Yaay!


To understand the flow of operations, run the below command:
>>> rqt_graph

