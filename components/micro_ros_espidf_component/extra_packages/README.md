# Extra ROS2 packages for microros
Place extra ROS2 packages here, they will get copied into the ROS workspace at build time.

The ESP-IDF has trouble calling clean on `make` based components, so changes to these packages may not get pulled into source on a rebuild.

If changes are not getting merged, try `make -f libmicroros.mk clean` within the microros component directory to force a rebuild of just this component.


## To add custom interface without re-downloading the entire micro-ROS source:

1) Copy custom ROS 2 interface package into here.

2) Delete only the micro-ROS build/install artifacts:
```
rm -rf components/micro_ros_espidf_component/micro_ros_src/install
rm -rf components/micro_ros_espidf_component/micro_ros_src/build
```
3) Run the standard build:
```
idf.py build
```