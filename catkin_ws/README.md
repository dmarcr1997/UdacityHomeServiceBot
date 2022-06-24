##Home Service Bot

	To run this simulation go to src/scripts and run ./home_service.sh
    This command will spin up the following launch files
    	* world.launch
        	** Starts up the simulation world in gazebo
            ** Spawns the robot into the world using the urdf node
            ** [URDF node docs](http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation)
        * view_navigation.launch
        	** Spins up the rviz robot simulator
            ** This launch file implements the rviz setting from the turtlebot package
            ** Write up rviz config used
        * amcl.launch
        	** Starts up the amcl node and the move_base node
            ** amcl node
            	*** amcl is a probabilistic localization package that implements Monte Carlo localization
                *** Monte Carlo localization uses particles and a filter algorithm to track the pose of the robot
                *** This type of localization needs a known map to determine the robot's pose
            ** move_base node
            	*** move_base allows for the control of the robots base to move it towards a goal location
                *** In this simulation it is given coordinates of a marker using a pub sub relationship
                *** This will be explained further below
  		* pick_objects
        	** This node subscribes to the marker location 
            ** Once it recieves data from the marker publisher it sends the bot to pickup the marker
            ** Then once it picks up the marker it waits 5 seconds, drops off the marker at the dropoff location
            ** This node implements params used by the marker class to determine when to hide/show the marker based off the robots location
        * add_markers
        	** This node will add a marker for the robot to pickup
            ** it implements the visualization_msgs package to show the marker in rviz
            ** There are two publishers in this node
            ** One for the visualization_msgs and another for the markers location which publishes a float32MultiArray that is used by the pick_objects node
            ** This node creates the marker, publishes its location, waits for the pick_objects param to be arrived_at_pickup, hides the marker, publishes the drop off location,
            then waits for the arrived_at_dropoff param, and finally returns
    ---
    The other launch files available implement various types of testing with the turtlebot package
    