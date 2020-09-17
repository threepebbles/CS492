This folder contains the URDF models and meshes for blocks and objects 
used for the experiments. 

This list will grow as more objects are added. For example, adding a box with 
as lid. 

Note that once Ein is used for visual servoing, there will be additional 
instance-based visual models or images.  


# How to create a new world model
We have two groups of files: world_file and world_xacro. The world_file is xml files that have been used to make a world model in the original system. The world_xacro is added to this project for the visualization purpose in RViZ. Here, we first explain how to add a primitive shape of object without meshes. 

## Add world_file and world_xacro if you have a new type of object except cube or cylinder
First, we add a xml file in following folder, 
```
cd data/worlds/
```
Then, create a xacro file in following folder,
```
cd data/models/xacro
```

## Add world_file and world_xacro that is a collection of object models to create a world model 
```
cd data/worlds/
```

You have to specify the orientation of objects using RPY or Quaternion. You can convert RPY to Quaternion (or Quaternion to RPY) using a following script in ipython,
```
import PyKDL
print PyKDL.Rotation.RPY(0,0,np.pi/4.0).GetQuaternion()
```

## Add world_file and world_xacro names in a pipeline launch file
```
cd launch
```

## Check the added object in RViZ
Launch RViZ as following.
```
rosrun rviz rviz
```
Add a RobotModel tab in Displays and then write down "world_description" on the Robot Description section




# Naming Rules
1. The name of root link in each object should end with 'base'
