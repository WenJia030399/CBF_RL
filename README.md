於 origin 含有 basic/final
basic 內為單純訓練
final 包含 low_level/high_level/logs
模型存放於 logs

low_level policy ：
        obs = np.array([
            *d_pos,
            dist,
            self.yaw,
            yaw_e
        ], dtype=np.float32)


於 gazebo_env 內
請動 gazebo 環境
```
ros2 launch my_gazebo gazebo_clean.launch.py world:=/home/wenjia/CBFRL/gazebo_env/uav_world.sdf gui:=true
```
開始 gazebo 訓練
```
python train.py 
```
強制關閉環境
```
ps aux | grep gz
killall -9 gzserver gzclient
```
