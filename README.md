## MuLtiUAV
一个基于ros的多无人仿真案例，其中具体包含两个demo，一个demo为多无人机护航仿真，另一个demo为视觉跟踪

### MultiUAV Convey

运行以下脚本实现无人机护航仿真
```bash
roslaunch MultiUAV multi_demo.launch
```
然后在新开的terminal中输入0解锁飞机，输入2启动板载模式，即可

### Visual Trick

运行以下脚本实现视觉跟踪
```bash
roslaunch MultiUAV trick_demo.launch
```
然后在新开的terminal中输入0解锁飞机，输入2启动板载模式，即可

若要移动小车，可采用以下脚本

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[-0.3,0,0]' '[0,0,0.0]'
```

