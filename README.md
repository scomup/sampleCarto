# sampleCarto

sampleCarto is a simplified version of the cartograher(https://github.com/googlecartographer), it is used for studying LiDAR-SLAM.  
(Much easier to understand than cartographer.^_^)  

The sampleCarto takes in a laser scans and wheel odometry, and outputs map.  
I have prepared a bag file (h1.bag), you can do some simple test.  

## usage:

mkdir build  
cd build  
cmake ..  
make 
./sampleCarto <path to your bag file>  

./rviz 
add the visualization for map topic and node_list topic.  
  
If your robot center is different from laser center, Please modify the following parameters in test.lua.  

<test.lua>  
TRAJECTORY_BUILDER_2D.baselink_to_laser_x = 0  
TRAJECTORY_BUILDER_2D.baselink_to_laser_y = 0  
TRAJECTORY_BUILDER_2D.baselink_to_laser_theta = 0  

