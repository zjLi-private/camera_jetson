# camera_bridge（K4A)

## 代码运行步骤

ROS环境下的编译及运行

1. 编译
    
    ```
    cd ~/camera_ws
    catkin_make --pkg camera_bridge -j
    ```
    
2. 注：若编译失败，记得清理上一次的build
    
    ```
    rm -rf build/camera_bridge devel/lib/camera_bridge
    ```
    
3. 编译成功后，可执行文件会放在：
    
    ```
    /camera_ws/devel/lib/camera_bridge/
    ```
    
4. 运行**(加载ROS环境)**
    
    ``` 
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    
    ```
    
    记得插上相机,观察白灯是否亮起
    
    ```
    rosrun camera_bridge k4a_detect_ros
    # or
    roslaunch camera_bridge k4a_and_serial.launch #带串口发送
    ```