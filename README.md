
# ROS2 Behavior Tree for Turtlesim

This repository demonstrates a simple implementation of a Behavior Tree (BT) in ROS2, using a simulated turtle (`turtlesim`). The behavior tree controls the turtle's actions based on its battery level and the environment. The turtle moves toward a charging station if the battery is low, charges itself, and then moves toward a goal if the battery is sufficiently charged.

## Table of Contents
- [Overview](#overview)
- [Behavior Tree](#behavior-tree)
- [Setup](#setup)
- [Running the Example](#running-the-example)

## Overview

This example shows how to integrate a Behavior Tree with a ROS2 node (`turtle_bt`) to control the turtle in `turtlesim` based on its battery status. The behavior tree includes actions for:
- **Checking the battery status**
- **Charging the battery**
- **Draining the battery**
- **Moving the turtle**
- **Changing the turtle's color based on battery level**

The turtle will:
- Move to a specific goal if the battery is above a certain threshold.
- Move to a charging station and charge itself if the battery is low.
- Display messages about the current battery status.
- Change its pen color based on the battery status (green for high, red for low).


## Behavior Tree

The behavior tree is defined in an XML file (`turtle_tree.xml`), where the turtle’s actions are arranged in a sequence or fallback structure. The root node of the tree is a fallback node that contains two sequences:
1. **High Battery Sequence**: If the battery is above 50%, it will:
   - Check the battery status.
   - Change the turtle's pen color to green.
   - Print a message.
   - Move to the target coordinates.
   - Drain some battery and print the current level.
2. **Low Battery Sequence**: If the battery is below 50%, it will:
   - Move to a charging station.
   - Charge the battery.
   - Change the pen color to red and print a message.

   ```xml
   <root BTCPP_format="4" main_tree_to_execute="MainTree">
   <BehaviorTree ID="MainTree">
      <Fallback name="root_fallback">
         <Sequence name="high_battery_sequence">
            <CheckBattery />
            <SetColorBasedOnBattery/>
            <SaySomething message="Battery is good!" />
            <MoveTo name="MoveToGoal" goal_x="0.2" goal_y="0.0" linear_speed="1.0" angular_speed="0.8" duration="10.0"/>
            <DrainBattery />
            <PrintBatteryLevel />
         </Sequence>
         <Sequence name="low_battery_sequence">
            <Inverter>
               <CheckBattery />
            </Inverter>
            <SetColorBasedOnBattery/>
            <MoveTo name="MoveToChargeStation" goal_x="0.2" goal_y="0.2" linear_speed="1.0" angular_speed="0.0" duration="2.0"/>
            <ChargeBattery />
            <SaySomething message="Battery charged!" />
            <PrintBatteryLevel />
         </Sequence>
      </Fallback>
   </BehaviorTree>
   </root>
   ```

## Setup

1. **Install ROS2**: This package was developed using ROS2. Make sure that you have ROS2 installed on your machine.

2. **Install Dependencies**:
   - `geometry_msgs`: Provides message types for motion commands.
   - `turtlesim`: Simulates a turtle in a 2D space.
   - `behaviortree_cpp`: Used for creating the behavior tree.

3. **Clone the Repository**:
   Clone this repository to your workspace and build the package.
   ```bash
   git clone https://github.com/sherif1152/Turtlesim_BT.git
   cd turtlesim_bt
   colcon build
   ```

4. **Source the Setup**:
   After building the package, source the workspace.
   ```bash
   source install/setup.bash
   ```

   ----

## Running the Example

 **Launch the Nodes**:
   Use the following command to launch the `turtlesim` and `turtle_bt` nodes:
   ```bash
   ros2 launch turtlesim_bt launch_turtlesim_bt.launch.py
   ```



<img src="turtlesim_bt.gif" alt="Turtlesim Behavior Tree Demo" width="800" />




