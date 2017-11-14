### Final Project - System Integration

The System Integration project is the final project of the Udacity Self-Driving Car Engineer Nanodegree. As a team, we built ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. This software system will be deployed on Carla (Udacity’s Self Driving Lincoln MKZ) to autonomously drive it around a test track.

### Team
| Name | Location | Project contribution |
| --- | --- | --- | 
|Prashant Gaikwad | Pune, India  | Team Lead, Planner subsystems: Waypoint publisher, node communication |
|Sanjay Shukla | Pune, India     | Perception subsystems: creating DL models, implementing tl_classfier  |
|Vikas Jethlya | Pune, India     | Perception subsystems: implementing tl_detector and publishing trafic waypoint |
|Mandar Pitley | Santa Clara, CA | Controler subsystem: Setup twist controller to follow waypoints |
|Carl Greg     | Santa Clara, CA |Controler subsystem: Implemented and tuned PID Controller |



### Software Architecture

Below are major module and their interfaces.

![Architecture](Reflection-images/architecture.png)

### Perception Subsystem:
This subsystem reads the world surrounding the vehicle and publishes relevant information to other subsystems. Specifically, this subsystem determines the state of upcoming traffic lights and publishes their status to other subsystems.


### Traffic Light Detection Node:

A crucial part of the vehicle’s self-driving capabilities comes from the ability to detect and classify upcoming traffic lights. This node processes images provided by the vehicle’s onboard camera and publishes upcoming traffic light information to the /traffic_waypoint topic. The Waypoint Updater node uses this information to determine if/when the car should slow down to safely stop at upcoming red lights. We take a two-stage deep learning based approach in traffic light classification. That is, the traffic light detection module consists of two CNN based models: traffic light detection (localization) and (light) color classification.


### Traffic Light Detection:
Traffic light detection takes a captured image as input and produces the bounding boxes as the output to be fed into the classification model. After many trial-and-errors, we decided to use TensorFlow Object Detection API, which is an open source framework built on top of TensorFlow to construct, train and deploy object detection models. The Object Detection API also comes with a collection of detection models pre-trained on the COCO dataset that are well suited for fast prototyping. We have used transfer learning becaus of limited data sets availability. I have experimented below pre-trained models with different data sets.
* faster_rcnn_resnet101_coco
* ssd_mobilenet_v1_coco

### Dataset used:
I have used the datasets which was shared in the forum for udacity simulator data approx 260 images and udacity real track data which is arrpund 160 images.

### Different steps taken for transfer learning:
Below are the different steps performed  to train pre-trained network using new datasets.

* Gathering the data, used udacity real and sim data shared on forum.
* Lable and annotate the datasets. 
* Creating Tfrecord file which would be used for training.
* Training the model and savining the model.
* Generating and saving graph for referencing.

The traffic-light detection is implemented in get_localization(self, image, visual=False) function in CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classifier.py. The traffic-light detector drops any camera frames that are older than 0.2 seconds to overcome any lag in the system.

### Performance Tuning:

The traffic light detection system was initially found to be always lagging 2-3 seconds behind the simulator despite improvements in prediction time and setting the queue size to one. This was due to a bug in how ROS processes large messages. The camera image subscriber was modified to have a large buffer size (52 Mb) and a queue size of one.

### Planning Subsystem:
The planning subsystem plans the vehicle’s path based on the vehicle’s current position and velocity along with the state of upcoming traffic lights. A list of waypoints to follow is passed on to the control subsystem.

### Waypoint Loader Node
This node was implemented by Udacity. It loads a CSV file that contains all the waypoints along the track and publishes them to the topic /base_waypoints. The CSV can easily be swapped out based on the test location (simulator vs real world).

### Waypoint Updater Node:
The bulk of the path planning happens within this node. This node subscribes to three topics to get the entire list of waypoints, the vehicle’s current position, and the state of upcoming traffic lights. Once we receive the list of waypoints, we store this result and ignore any future messages as the list of waypoints won’t change. This node publishes a list of waypoints to follow - each waypoint contains a position on the map and a target velocity.

### Control Subsystem:
This subsystem publishes control commands for the vehicle’s steering, throttle, and brakes based on a list of waypoints to follow.

### Waypoint Follower Node:
This node was given to us by Udacity. It parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic.

### Drive By Wire Node:
he DBW node is the final step in the self driving vehicle’s system. At this point we have a target linear and angular velocity and must adjust the vehicle’s controls accordingly. In this project we control 3 things: throttle, steering, brakes. As such, we have 3 distinct controllers to interface with the vehicle.

### Throttle Controller:
The throttle controller is a simple PID controller that compares the current velocity with the target velocity and adjusts the throttle accordingly. The throttle gains were tuned using trial and error for allowing reasonable acceleration without oscillation around the set-point.

### Steering Controller:
This controller translates the proposed linear and angular velocities into a steering angle based on the vehicle’s steering ratio and wheelbase length. To ensure our vehicle drives smoothly, we cap the maximum linear and angular acceleration rates. The steering angle computed by the controller is also passed through a low pass filter to reduce possible jitter from noise in velocity data.

### Braking Controller:

This is the simplest controller of the three - we simply proportionally brake based on the difference in the vehicle’s current velocity and the proposed velocity. This proportional gain was tuned using trial and error to ensure reasonable stopping distances while at the same time allowing low-speed driving

### Testing:
Most of the testing was done using the Udacity simulator. But we also tested our system on the site rosbag files, using the provided site.launch file. For example:

#launch site launch file roslaunch ros/launch/site.launch #play bag file rosbag play -l


### Known Issues and Future Scopes



