# opencv_node

A ROS node which publishes and subscribes OpenCV images. There is the possibility for imread, imwrite and imshow images with OpenCV. 

## Installation

You have to go to the source directory of your `catkin workspace` and clone it:

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/opencv_node.git
cd ~/catkin_ws
catkin_make
```

## Usage

### Read and publish an image

```
rosrun opencv_node imread.py
```

### Subscribe and write an image

```
rosrun opencv_node imwrite.py
```

### Subscribe and show an image

```
rosrun opencv_node imshow.py
```
