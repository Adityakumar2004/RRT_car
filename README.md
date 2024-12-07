# RRT_car

- This repository tries to replicate the RRT for a car like robot whose motion is contrained
- this [paper](https://www.sciencedirect.com/science/article/pii/S1474667015343603#:~:text=Abstract,a%20control%20input%20selection%20approach) is closely followed for the implementation 

## how to run

- run this command to start the slam algorithm which is from the gmap package of ros-1 noetic<>
<code>roslaunch custom_navigation gmapping_custom_param.launch</code>

- path of the map obtained needs to be given in the [inflation_layer.py](/custom_navigation/src/scripts/inflation_layer.py)
