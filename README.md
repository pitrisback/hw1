# Homework1 - Perception	

### Task 1	

0) Start gazebo, add some objects on the table and start the AprilTag detection node
```bash
roslaunch ar_arena ar_bringup.launch simulation:=true
roslaunch ar_arena apriltag.launch simulation:=true
```

1) Run the AprilTag node and request some objects to be analyzed

```bash
catkin build hw1 && rosrun hw1 april_parse red_cube_1 red_prism_5
```

`red_prism_5` is not a valid `frame_id` so it will be safely ignored.

The position and orientation of the object will be saved in `output/pose_<id>.txt`

### Task 2

0) Start gazebo, add some objects on the table

1) Run the PCL node and request some categories of objects to be searched for

```bash
catkin build hw1 && rosrun hw1 pcl_find_objects red_cube blue_cube blue_cube
```

The number of objects found is printed

```bash
[ INFO] [1584575652.426455591, 4173.393000000]: Trovati 6 oggetti
```

For each object, the category and the position (with z relative to the table) is printed

```bash
[ INFO] [1584575652.427272921, 4173.393000000]: Analizzo object 0
[ INFO] [1584575652.428928462, 4173.394000000]: Trovata forma yellow_cyl, pose: -0.332989 -0.144777 0.084070
```


In a scene where one red cube and only one blue cube are present, the following output would be produced.

```bash
[ INFO] [1584575509.148890444, 4116.524000000]: Trovato red_cube
[ INFO] [1584575509.148902247, 4116.524000000]: Trovato blue_cube
[ INFO] [1584575509.148914350, 4116.524000000]: Non ho trovato blue_cube
```
