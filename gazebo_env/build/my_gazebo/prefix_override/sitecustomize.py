import sys
if sys.prefix == '/home/wenjia/miniconda3/envs/rl':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wenjia/CBFRL/gazebo_env/install/my_gazebo'
