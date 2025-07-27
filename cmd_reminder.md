1. start ignition gazebo
    ```
    ign gazebo empty.sdf
    ```
    it will launch the default empty world. Change the sdf file to your own world file like this
    ```
    ign gazebo -v 4 -r amazeworld.sdf
    ```

2. import .urdf file to the gazebo environment (especially empty.sdf)
    ```
    ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "path/to/your/urdf/file.urdf", name: "urdf_model"'
    ```

3. start the RViz2 visualization tool
    ```
    source /opt/ros/humble/setup.bash

    rviz2

    the use RViz2 according to reference 3
    ```

4. bridge ignition and ros2 topics
    ```
    ros2 run ros_gz_bridge parameter_bridge /model/amazebot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
    ```

5. publish ignition topics
    ```
    ign topic -t /model/amazebot/cmd_vel -m ignition.msgs.Twist --mode publish -p "linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.1}"
    ```

6. echo ignition topics
    ```
    ign topic -e -t /model/amazebot/cmd_vel
    ```

7. publish ros2 topics
    ```
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1, y: 0, z: 0} angular: {x: 0, y: 0, z: 0.1}"
    ```

8. echo ros2 topics
    ```
    ros2 topic echo /cmd_vel
    ```

9. where are all the plugins?
    ```
    cd /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
    ```

10. using keyboard to publish 'cmd_vel' topic
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/amazebot/cmd_vel
    ```