## HTTP手动控制天工机器人 🤖
此部分包含天工2.0plus和天轶2.0Pro。

![Operation](README_source/20251211-093833.jpg)


步骤：
1. 修改 src/robot_control/robot_control/http_control.py 中的数据库以及dist路径。

2. cd /path_to/http_to_ros
    ```bash
    colcon build --symlink-install
    ```
3. 运行:
    ```bash
    cd /path_to/http_to_ros
    export ROS_DOMAIN_ID=0
    source install/setup.bash
    ros2 run robot_control http_conrtol
    ```

ps:

天工2.0系列机器人开箱机器人需要切换到motion_control模式下

天轶天工在未配置DDS情况下：

 天轶2.0系列机器人需要将该程序部署在内部

 天工2.0系列则可分布式通信（连接网线）

联系作者：86+ 19987400216

=======================================

更新日志：

2025.12.23

动作组配置已完成

动作组/动作执行已完成

动作录制起步

新增电流
