# Challenge handin for Hugo

## Setup
`mkdir ros2_ws`  
`cd ros2_ws`  
`mkdir src`  
`cd src`  
`git clone git@github.com:Ozmehak/challenge_handin.git`  
`cd ..`  
`vcs import src < src/challenge_handin/dependencies.repos`  
`rosdep install --from-paths src -y --ignore-src`  

## Patch
`cd src/challenge_handin/patches/`  
`chmod +x autopatcher.py`   
`./autopatcher.py`

## Build
`cd ../../`
`colcon build`  
`source install/setup.bash`  

## Run
`ros2 launch challenge_handin challenge_handin.launch.py`  
`You can now play the rosbag file and enjoy the robots movement`
`Also if you change fixed frame to "base_link" instead of "odom" rviz you can see the scans`

## Thoughts and Conclusions

I started out with ros2 iron irwini, went with the diff_drive_controller route and ran into problems, no matter what I tried I could not get it up and running.  
So I went the MathRoute and my plots are in the img folder, however they did not feel correct, something is off with my Math I think, script file is "robot_plotting.py in the scripts folder.     
After getting stuck with the Math issues I changed approach, went back to Humble, managed to get the controller up and running swiftly and you can see my results in rviz.
Time was runnin short so I decided to give you what I got so far, you have to swap between base_link and odom in rviz to see the movement and the scans.      
I did not try the Kalman filter yet, but I will.  
Overall a very fun test!    
