# avatar_challenge

## Docker Image
To run the system using Docker and access the graphical desktop:
   'docker build -t aare-sushma/avatar_challenge'. Run the container using name and RDP access: 'docker run --name xarm-challenge-v2 --platform linux/amd64 -p 5567:3389 aare-sushma/avatar_challenge'. Use Remmina Connect to: 'localhost:5567'. 
   Login credentials:
    Username: dev
    Password: as provided by Avatar Robotics

## Developing
The ROS 2 package is located at: '/home/dev/dev_ws/src/avatar_challenge'. To launch the system, open a terminal inside the Docker container and run: 'cd ~/dev_ws' and 'ros2 launch avatar_challenge start.launch.py'. This will open the RViz window and wait for user input. If the launch file does noot work, 'source /opt/ros/humble/setup.bash', 'source ~/xarm_ws/install/setup.bash', 'source ~/dev_ws/install/setup.bash' and try again
Add a MarkerArray display in RViz and the topic should be '/visualization_marker_array'. Uncheck the Motion Planning and Trajectory checkboxes in the Displays panel for a cleaner visualization.
Press enter in the docker terminal and we can see the arm drawing different shapes on Rviz platform

## Input Shape Definitions
All shapes are defined in a JSON file located at /home/dev/dev_ws/src/avatar_challenge/config/shapes.json
Each shape entry includes
    type: One of "polygon", "arc", or "bspline"
    start_pose: Defines the 3D plane for drawing, with:
    position: [x, y, z] coordinates in meters
    orientation_rpy: [roll, pitch, yaw] in radians
    Shape-specific parameters:
        For polygon: "vertices" – list of 2D [x, y] points
        For arc: "center", "radius", "start_angle", "end_angle", "segments"
        For bspline: "control_points", "degree", "num_points"
To add new shapes:
    Edit shapes.json
    Add a new shape object to the "shapes" list following the format above. Save and rerun the launch file inside the container.

## Approch
The project uses MoveIt 2 and RViz to control a simulated xArm7 for drawing 2D shapes projected onto 3D planes in space. The idea is to define shapes in a 2D coordinate system and then transform them onto a plane using a 3D start_pose with position and orientation. 
The software supports multiple shape types—polygons, arcs, and B-splines—allowing for both sharp-edged and smooth paths. For polygons, linear interpolation is added between corners to smoothen transitions, while B-spline curves are generated using control points and degree-based basis functions. 
RViz MarkerArrays are used to visualize the planned paths before execution. Cartesian path planning is used for trajectory generation, with velocity and acceleration scaling to ensure smoother and more realistic motion. The whole system is easy to customize through a JSON file and works for any shape drawn on a flat surface in 3D space.
To approach the problem, I broke it down into several key steps: parsing the shape data from the JSON file, creating the MoveIt interface, converting 2D points to 3D poses based on the given plane, and finally moving the arm to the start pose and executing the trajectory. One of the challenges I encountered was related to loading the robot's semantic description. When I added draw_shapes_node to the launch file, the node failed due to missing SRDF (robot_description_semantic) parameters. I resolved this by explicitly retrieving and declaring these parameters within the code. Additionally, since the xarm7_moveit_fake.launch.py file already initializes RViz, I had to carefully work around RViz startup behavior and manually initialize the marker publisher within the node to ensure the drawing was visible during runtime.



