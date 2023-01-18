# Record Synchonrized Data (Given Topics) 

## Installation
Build the package
```cd ~/catkin_ws && catkin_make -j3```

### Run (taking the Apollo platform as an example)
Run the callDataOnline_node_apollo.cpp
```rosrun data_collection callDataOnline_apollo \
    data_path \
    lidar_topic \
    frame_cam00_topic \
    frame_cam01_topic \
    event_cam00_topic \
    event_cam01_topic
```
Or
```
./script/save_fp_data.sh data_path
```

Publish a service that the code can react
```rosservice call /data_record "{}"```