# Homework1 - Perception
## Task 1

Suppose to have N=16 objects identified by N different [APRILTAG](https://april.eecs.umich.edu/software/apriltag) markers (id = 0, ..., 15):
- Only n ≤ 16 objects will be randomly placed on the table in front of the robot;
- From command line, a human operator will ask for the recognition of x ≤ n of them. The request will
be made through frame_id:
```
$ rosrun your_package your_node frame_id_1 frame_id_2 ...
```
##### Note:
- The APRILTAG library detects the markers through their ids;
- The library outputs the pose (position and orientation) of each marker.

##### Task 1 goal:
Implement a NODE able to:
- Match the id of an object with its frame_id;
- Accept inputs from command line;
- Read a /topic publishing the pose of each requested object;
- Store these poses on a text file.

##### Apriltags example:
- Open a shell and launch the simulated environment in Gazebo:
```
$ roslaunch ar_arena ar_bringup.launch simulation:=true
```
- Place some objects on the table in front of the robot and, on a new shell, launch the AprilTag node:
```
$ roslaunch ar_arena apriltag.launch simulation:=true
```
- Open Rviz (third shell):
```
$ rviz
```
and ADD the topic /tag_detections (You can visualize the published information also by command line: rostopic echo /tag_detections – or launching rqt_image_view and selecting the above-mentioned topic)

## Task 2

As before, n objects are on the table (yellow cylinders, red and blue cubes, red and green prisms). From
command line, a user ask for the detection of x ≤ n of them, naming their categories:
```
$ rosrun your_package your_node red_cube red_cube yellow_cylinder ...
```
##### Homework goal:

Implement a NODE that exploits the [Point Cloud Library] (PCL - http://pointclouds.org/) to detect each
object. The adopted algorithm should exploit colors or shapes in order to recognize objects. E.g., one
known PCL 3D object recognition technique is based on [Correspondence Grouping]
(http://pointclouds.org/documentation/tutorials/correspondence_grouping.php)

##### Note:
- Object meshes are provided (in ar_arena/meshes)
