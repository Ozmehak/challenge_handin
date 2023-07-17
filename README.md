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

## Build
`colcon build`  
`source install/setup.bash`  

## Run
`ros2 launch challenge_handin challenge_handin.launch.py`  