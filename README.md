# Spider-Camera
UCL Computer Vision, Graphics and Imaging MSc Thesis

## ROS Nodes Description
### bbox_to_cmd_vel 
Computes velocity commands for all 4 motors from Bounding Box location  
Subscribes to `/bbox`  
Publishes `(vel_1, vel_2, vel_3, vel_4)` on `/cmd_vel`  

### siam_tracker 
Real-time Tensorflow Tracker (based on SiamFC by Bertinetto et al.)  
Publishes `(x,y,w)` on `/bbox`  
