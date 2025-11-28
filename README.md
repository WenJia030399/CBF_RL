```
ros2 launch my_gazebo gazebo_clean.launch.py world:=/home/wenjia/CBFRL/gazebo_env/uav_world.sdf gui:=true
```
```
python train.py 
```
```
ps aux | grep gz
killall -9 gzserver gzclient
```
