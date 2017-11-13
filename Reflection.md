### Software Architecture

Below are major module and their interfaces.

![Architecture](Reflection-images/architecture.png)

### Perception Subsystem
This subsystem reads the world surrounding the vehicle and publishes relevant information to other subsystems. Specifically, this subsystem determines the state of upcoming traffic lights and publishes their status to other subsystems.


### Traffic Light Detection Node

A crucial part of the vehicle’s self-driving capabilities comes from the ability to detect and classify upcoming traffic lights. This node processes images provided by the vehicle’s onboard camera and publishes upcoming traffic light information to the /traffic_waypoint topic. The Waypoint Updater node uses this information to determine if/when the car should slow down to safely stop at upcoming red lights. We take a two-stage deep learning based approach in traffic light classification. That is, the traffic light detection module consists of two CNN based models: traffic light detection (localization) and (light) color classification.


### Traffic Light Detection
Traffic light detection takes a captured image as input and produces the bounding boxes as the output to be fed into the classification model. After many trial-and-errors, we decided to use TensorFlow Object Detection API, which is an open source framework built on top of TensorFlow to construct, train and deploy object detection models. The Object Detection API also comes with a collection of detection models pre-trained on the COCO dataset that are well suited for fast prototyping. Specifically, we use a lightweight model: ssd_mobilenet_v1_coco that is based on Single Shot Multibox Detection (SSD) framework with minimal modification. The COCO dataset contains images of 90 classes ranging from vehicle to human. The index for traffic light is 10. Though this is a general-purpose detection model (not optimized specifically for traffic light detection), we find this model sufficiently met our needs, achieving the balance between good bounding box accuracy (as shown in the following figure) and fast running time.

The traffic-light detection is implemented in get_localization(self, image, visual=False) function in CarND-Capstone/tree/master/ros/src/tl_detector/light_classification/tl_classifier.py. The traffic-light detector drops any camera frames that are older than 0.2 seconds to overcome any lag in the system.

### Traffic Light Classification


### Performance Tuning

The traffic light detection system was initially found to be always lagging 2-3 seconds behind the simulator despite improvements in prediction time and setting the queue size to one. This was due to a bug in how ROS processes large messages. The camera image subscriber was modified to have a large buffer size (52 Mb) and a queue size of one.

Another possible bottleneck was found to be in the very-first image processing step which was somehow very slow compared to the later ones. To overcome this, during the initialization of the tl_detector node, the traffic light localizer/classifier is called with an empty 800x600 image ensuring that all the TensorFlow systems are initialized. This ensured that subsequent evaluations were much faster.
