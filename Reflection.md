
## Final Project - System Integration

The System Integration project is the final project of the Udacity Self-Driving Car Engineer Nanodegree. As a team, we built ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. This software system will be deployed on Carla (Udacity’s Self Driving Lincoln MKZ) to autonomously drive it around a test track.

## Team - Rock On
| Name | Location | Project contribution |
| --- | --- | --- | 
|Prashant Gaikwad | Pune, India  | Team Lead, Planner subsystems: Waypoint publisher, node communication |
|Sanjay Shukla | Pune, India     | Perception subsystems: creating DL models, implementing tl_classfier  |
|Vikas Jethlya | Pune, India     | Perception subsystems: implementing tl_detector and publishing trafic waypoint |
|Mandar Pitale | Santa Clara, CA | Controler subsystem: Setup twist controller to follow waypoints |
|Karl Greb     | Santa Clara, CA |Controler subsystem: Implemented and tuned PID Controller |


## Software Architecture

Below are major module and their interfaces.

![Architecture](Reflection-images/architecture.png)

There are 3 major block  
1. Perception Subsystem  
2. Planning Subsystem  
3. Control Subsystem  

Detail explanation is below:

### 1. Perception Subsystem:
This subsystem reads the world surrounding the vehicle and publishes relevant information to other subsystems. Specifically, this subsystem determines the state of upcoming traffic lights and publishes their status to other subsystems.


#### 1.1 Traffic Light Detection Node:

A crucial part of the vehicle’s self-driving capabilities comes from the ability to detect and classify upcoming traffic lights. This node processes images provided by the vehicle’s onboard camera and publishes upcoming traffic light information to the /traffic_waypoint topic. The Waypoint Updater node uses this information to determine if/when the car should slow down to safely stop at upcoming red lights. We take a two-stage deep learning based approach in traffic light classification. That is, the traffic light detection module consists of two CNN based models: traffic light detection (localization) and (light) color classification.

#### 1.2 TL detector :

The traffic light detection node (tl_detector.py) subscribes to three topics:

* /base_waypoints : provides the complete list of waypoints for the course.  
* /current_pose can be used to determine the vehicle's location.  
* /image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.  

The node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic. This index will be later be used by the waypoint updater node to set the target velocity. 

* /traffic_waypoint

Modification done in tl_detector.py :
* New Function added:  
    * distance(self, pos1, pos2)
    * get_closest_waypoint(self, pose)
    * distance_light(self, lp, wp)
    * get_closest_waypoint_light(self, wp, lp)  
* Modified Function :  
    * __init__(self):
    * process_traffic_lights(self)
    
#### 1.3 Traffic Light Classification:
Traffic light detection takes a captured image as input and produces the bounding boxes as the output to be fed into the classification model. After many trial-and-errors, we decided to use TensorFlow Object Detection API, which is an open source framework built on top of TensorFlow to construct, train and deploy object detection models. The Object Detection API also comes with a collection of detection models pre-trained on the COCO dataset that are well suited for fast prototyping. We have used transfer learning becaus of limited data sets availability. We have experimented below pre-trained models with different data sets.
* faster_rcnn_resnet101_coco
* ssd_mobilenet_v1_coco

#### 1.4 Dataset used:
We have used the datasets which was shared in the forum for udacity simulator data approx 260 images and udacity real track data which is arrpund 160 images.

#### 1.4 Different steps taken for transfer learning:
Below are the different steps performed  to train pre-trained network using new datasets.

* Gathering the data, used udacity real and sim data shared on forum.
* Lable and annotate the datasets. 
* Creating Tfrecord file which would be used for training.
* Training the model and savining the model.
* Generating and saving graph for referencing.

The traffic-light detection is implemented in get_classification(self, image) function in CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classifier.py.

Modification done in tl_classifier.py :
* Modified Function :  
    * __init__(self, real):
    * get_classification(self, image)

#### 1.5 Performance Tuning:

The traffic light detection system was initially found to be always lagging 2-3 seconds behind the simulator despite improvements in prediction time and setting the queue size to one. This was due to a bug in how ROS processes large messages. The camera image subscriber was modified to have a large buffer size (52 Mb) and a queue size of one.


#### 1.6 Location of TL Classifier DNN Model:   [Click Here](https://drive.google.com/drive/folders/1fTJDygmGa9YIMmjtKWedeNDJNla1rYVx?usp=sharing)  
It contains the sim and real track model of faster_rcnn_resnet trained with Udacity real car track and simulator dataset. 

### 2. Planning Subsystem:
The planning subsystem plans the vehicle’s path based on the vehicle’s current position and velocity along with the state of upcoming traffic lights. A list of waypoints to follow is passed on to the control subsystem.

#### 2.1 Waypoint Loader Node
This node was implemented by Udacity. It loads a CSV file that contains all the waypoints along the track and publishes them to the topic /base_waypoints. The CSV can easily be swapped out based on the test location (simulator vs real world).

#### 2.2 Waypoint Updater Node:
The bulk of the path planning happens within this node. This node subscribes to three topics to get the entire list of waypoints, the vehicle’s current position, and the state of upcoming traffic lights. Once we receive the list of waypoints, we store this result and ignore any future messages as the list of waypoints won’t change. This node publishes a list of waypoints to follow - each waypoint contains a position on the map and a target velocity.

### 3. Control Subsystem:
This subsystem publishes control commands for the vehicle’s steering, throttle, and brakes based on a list of waypoints to follow.

#### 3.1 Waypoint Follower Node:
This node was given to us by Udacity. It parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic.

#### 3.2 Drive By Wire Node:

The dbw_node is responsible for taking the input twist_cmd topic and delivering the output brake_cmd, steer_cmd, and throttle_cmd topics.  The initial task for dbw_node was subscribing to the input topics and publishing the output topics.  Once this was done, we could focus on writing the control loops necessary for the vehicle dynamics to work.  

The steering is managed through a provided control algorithm called yaw_control. The yaw_control algorithm is simple, and mostly used to convert the proposed angular velocity into a steering command, adjusting for the current velocity and the test vehicle characteristics (e.g. steering ratio and wheel base).  To further reduce any spikes in steering output commands, a low pass filter is used to smooth the output before it is sent to the steering ECU.  

The throttle and brake commands are managed through a slightly more complicated approach. First, we use a PID control loop to try to ensure that the current velocity is equivalent to the proposed velocity. A low pass filter is then used to average values on the acceleration and smooth out any sharp transients. The subsequent acceleration value is then split into a throttle command or a brake command based on whether the value is greater than zero (accelerate) or less than zero (brake).  The provided brake deadband value is used in a further filter on noise in braking.  For brake commands there is a further calculation done to translate the percentage of braking desired into N-m of braking force, as required by MKZ vehicle fs braking system.  

We had a lot of challenges in tuning the PID and LPF parameters.  The simulator is highly demanding on target HW and we saw many times when we could not get the same performance on two different target machines using the same software with the same parameters.  Also we found many instances of glitching in the simulator which we could not explain.  Ultimately we decided to set our PID and LPF parameters empirically working on a single machine for simulation in a single set of use cases.  Due to the numerous issues in the simulator we expect that further real-world tuning would be needed to improve the calibration of the control algorithms.  

We also tried to use the provided rosbag and dbw_test node for tuning our performance. This testing confirmed that our control outputs were happening in the right time and following the right trends, but it was not possible to attempt a 1:1 match due issues in the bag file values.  The throttles values appeared to be capped at 2.5% of vehicle maximum throttle.  The brakes values were not converted to N*m but reported natively.  The steering was inherently different due to differences in the velocity because of the capped acceleration values.  This is an area where Udacity needs to make major improvements for future students.


## Testing:
Most of the testing was done using the Udacity simulator. But we also tested our system on the site rosbag files, using the provided site.launch file. For example:

#launch site launch file roslaunch ros/launch/site.launch #play bag file rosbag play -l

* We tested on simulator for 8 hours and it works fine. 
* Final video of car running on simulator track : [Click Here](https://www.youtube.com/watch?v=K2zwgdYkpaY&feature=youtu.be)



## Known Issues :
* Sometimes simulator is not able to connect with ros nodes. Car fails to move on track. As a work around, need to restart the simulator and launch the ros.
* Car is always stopping at traffic light signal irrespective of traffic signal light. And once signal is green, it moves.
