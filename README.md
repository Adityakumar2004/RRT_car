# RRT_car

- This project implements RRT for a car like robot which is non-holonomic i.e., it has got 2 controls for 3 degree of freedom space
- this [paper](https://www.sciencedirect.com/science/article/pii/S1474667015343603#:~:text=Abstract,a%20control%20input%20selection%20approach) is closely followed for the implementation
## how to run

<<<<<<< HEAD
- run this command to start the slam algorithm which is from the gmap package of ros-1 noetic <br>
=======
- run this command to start the slam algorithm which is from the gmap package of ros-1 noetic<>
>>>>>>> 13fac0a285cb61ed708d648cd1f8ed46c897edaa
<code>roslaunch custom_navigation gmapping_custom_param.launch</code>

- path of the map obtained needs to be given in the <I>[inflation_layer.py](/custom_navigation/src/scripts/inflation_layer.py) </I><br>
this creates an inflated map a map where the collision of the mobile robot with the walls is taken into account by increasing the thickness of the walls with appropriate dimensions specific to the car model
- pass this map to the rrt_final.py
- from RVIZ choose your intial and goal pose and the rrt_final.py displays the trajectory on the rviz. The trajectory is found such that the non holonomic constrained is taken into account.

- this is how the path looks like there are few changes which are needed to avoid multiple loops as can be seen in the image below
- 
