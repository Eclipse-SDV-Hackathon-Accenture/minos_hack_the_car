
# Hackathon
Thi is a repo for Hackathon Hack the car challenge. This project focuses on retrieving a 3d point cloud from the LiDAR sensor in order to predict bumps in front of the car.

## Author
* Artur
* Yang
* Ishan
* Ayush
* Matteo

## Setup
### Needed python libraries:
* open3d
* pointcloud2
* PIL
* numpy
* ecal
* 

### Software & interface
* eCAL
* Foxglove
* ecal-foxglove-bridge

### How to run
* for offline mode, you need to prepare recorded data file, start eCAL player and play the data
* for on line mode, connect your PC to device to get real time data
* start eCAL mornitor, topic names and message types are presented
* start Foxglove and ecal-foxglove-bridge
```shell
python ecal_foxglove_bridge
```
* run protobuf_rec.py to subscribe data and detect barriers and bumps on the road surface

## Structure
### Communication
The communication interface and data flow is shown as below:


### Algorithm
Refer to our ... to find out more


## Result
* Foxglove visualization
![image](https://github.com/Eclipse-SDV-Hackathon-Accenture/minos_hack_the_car/img/foxglove_window.png)

* ground points segmentation
![image](https://github.com/Eclipse-SDV-Hackathon-Accenture/minos_hack_the_car/img/1st_ground.png)

* point cloud clustering
![image](https://github.com/Eclipse-SDV-Hackathon-Accenture/minos_hack_the_car/img/1st_res.png)

* bump detection
![image](https://github.com/Eclipse-SDV-Hackathon-Accenture/minos_hack_the_car/img/1st_bump.png)


