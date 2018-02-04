# sampleCarto

sampleCarto is a simplified version of cartograher(https://github.com/googlecartographer), it is used for studying LiDAR-SLAM.
(Much easier to understand than cartographer.)

The sampleCarto takes in a laser scans and wheel odometry, and outputs map.
I have prepared a bag file (h1.bag), you can do some simple test.

## usage:

mkdir build
cd build
cmake ..
make 
./sampleCarto

./rviz 
add the visualization for map topic and node_list topic.
