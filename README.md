# Competition 5: Game Strategy

**Description:** In this competition, the robot is supposed to follow a line, count objects, match similar shaped objects, find AR tags and push a box with AR tags to an AR tag goal location. We were supposed to pick a stragtegy that gives us the most mark. Our original strategy was to complete all the tasks all the time and start the robot again once it's finished. Then we learned during the competition that we are not allowed to manually start the robot ourselves so we ended up just let the robot finish one complete run.

## 1. Purpose

For this competition, we want the robot to follow a pre-defined course and do different vision and navigation tasks. 

### The Course Map

- The course map in simulation:

    <img src="https://github.com/CMPUT412W19Team6/Comp3/blob/master/course.PNG" width="400">

- The actual course's picture:

    <img src="https://github.com/CMPUT412W19Team6/Comp3/blob/master/course_real.jpg" width="200">
<!-- ![pick link](https://github.com/CMPUT412W19Team6/Competition2/blob/master/course_pic.png?s=200) -->

- Explanation:
     - The robot is supposed to follow the white lines on the floor, and stop for a bit when reaching a red mark on the floor.
     - There are 3 locations (1,2 and 4) where vision tasks are required in addition to another location (3) that requires both vision and navigation tasks:
        
        a. Location 1: 
        
        Detect how many cylinders are on the left , (Maximum 3), and signal the result with LED lights and sound. 
        Example picture:         
        <img src="https://github.com/CMPUT412W19Team6/Competition2/blob/master/location1.png?s=200" width="200">
        <!-- ![location 1 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location1.png?s=200) -->
        
        b. Location 2: 
        
        * Detect how many geometric shapes are there (Maximum 3), signal the result with LED lights and sound,and recognize what the green shape is (one of the three shapes: triangle, square, circle). 
        Example picture:         
        <img src="https://github.com/CMPUT412W19Team6/Competition2/blob/master/location2.png?s=200" width="200">
        
        <!-- Example picture: ![location 2 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location2.png?s=200) -->
         c. Location 3: 
        
         * Recognize the red shapes on the left one by one, and signal with a sound when finding one that's matching the green shape discovered at Location 2. 
        Example picture:         
        <img src="https://github.com/CMPUT412W19Team6/Competition2/blob/master/location3.png?s=200" width="200">
        <!-- Example picture: ![location 3 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location3.png?s=200) -->
        
        d. Location 4: 
          > Note1: The box must has AR tag id=1 from from this [link](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers0to8.png) and the AR tag for the goal position must have a different tag id from the box which is less than 9.
          
          > Note2:Parking stalls, against the back wall, are numbered (counting from the window-on-the-left) #1, 2, 3, 4, & 5. The box will only be in one of #2,3,4. The goal position will be in any of #1 to #5 that doesn't has a box. The Shape will be in the squares that's `NOT` any of #1,2,3,4,5.
           
        * Find the goal position marked by an AR tag and signal a sound and a green LED light
        	
            <img src="https://github.com/CMPUT412W19Team6/Comp3/blob/master/ar.jpg" width="200" title="AR tag">
           
        * Find AR tag wrapped box and signal a sound and red LED light.
        
            <img src="https://github.com/CMPUT412W19Team6/comp5/blob/master/box.png" width="200" title="AR box">
        
        * Find and Park at the spot that has the object with the same shape as the green shape from Location 2, signal sound and orange LED light when found and signal sound, origin and green LED lights when parked.
			
            <img src="https://github.com/CMPUT412W19Team6/Comp3/blob/master/object.jpg" width="200" title="shape">
        
        * Push the Box into the goal spot identified by the AR marker
        

## 2. Pre-requisites

### 2.1 Hardware requirement

- A kobuki turtlebot base pack
  - Robot base
  - Middle Plate
  - 39mm Poles *6
  - 50mm/39mm Poles *4
  - 83mm Poles *2
  - C-Clamps for holding the Xtion Pro camera
  - Asus Xtion Pro Plate
  - A block of Foam for better box pushing effect
- A USB Camera
- An Asus Xtion Pro
- A controller (prefered logitech)
- A device with minimum 4 usb ports
- The course set up 

#### 2.11 Kobuki base set up

  <img src="https://github.com/CMPUT412W19Team6/comp5/blob/master/robot1.jpg" width="200" title="robot1">
  <img src="https://github.com/CMPUT412W19Team6/comp5/blob/master/robot2.jpg" width="200" title="robot2">
  <img src="https://github.com/CMPUT412W19Team6/comp5/blob/master/robot3.jpg" width="200" title="robot3">
  <img src="https://github.com/CMPUT412W19Team6/comp5/blob/master/robot4.jpg" width="200" title="robot4">

  1. Combine 4 39mm poles and 4 50mm/39mm poles together.
  2. Install the combined poles onto the base as shown in the above pictures.
  3. Place the Middle Plate ontop.
  4. Clap the Xtion Pro on the Middle Plate at the front of the robot.
  5. Install 2 83mm poles behind the Xtion pro as shown in the above pictures.
  6. Place the Xtion Pro Plate on top of those 2 poles.
  7. Place the usb camera on the Xtion Pro Plate and make it face 45 degrees downwards. Tape if if necessary.
  8. Install 2 39mm poles at on the base below the Middle Plate.
  9. Put the Foam against these 2 poles and fix it by the C-Clamp.
  
### 2.2 Software requirement
> Note: The navigation task did not use any map or amcl. It's purely based on odom.

- ROS kinetic (which includes opencv2 ros package) ([Guide here](http://wiki.ros.org/kinetic/Installation/Ubuntu))

- Turtlebot packages ([Guide here](http://wiki.ros.org/action/show/Robots/TurtleBot?action=show&redirect=TurtleBot))

  ```bash
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-turtlebot-apps
  sudo apt-get install ros-kinetic-turtlebot-interactions
  sudo apt-get install ros-kinetic-turtlebot-simulator
  ```

- Kobuki ROS packages ([Guide here](https://wiki.ros.org/kobuki/Tutorials/Installation))

  ```bash
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  ```

- Upgrade camera packages

  ```bash
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  ```

- ROS smach ([Guide here](http://wiki.ros.org/smach))

- Python2.7 ([Guide here](https://www.python.org/))

- Python package `imutils` ([Repo here](https://github.com/jrosebr1/imutils))
  ```
  pip install imutils
  ```
  
- rviz ([Wiki here](http://wiki.ros.org/rviz))
  ```
  sudo apt-get install ros-kinetic-visualization
  ```
  
- ar_track_alvar ([Wiki here](http://wiki.ros.org/ar_track_alvar))
  ```
  sudo apt-get install ros-kinetic-ar-track-alvar
  ```
- usb_cam ([Wiki here](http://wiki.ros.org/usb_cam))
  ```
  sudo apt-get -install ros-kinetic-usb-cam
  ```
## 3. Execution

### 3.1 Quickstart
0. Set up the court and your robot base as described earlier.

1. Clone this repo into the source directory of your catkin workspace (e.g. catkin_ws/src)

   ```bash
   # under catkin_ws/src folder
   mkdir comp5
   git clone https://github.com/CMPUT412W19Team6/comp5.git comp5
   ```

2. Run catkin_make and source the setup.bash

   ```bash
   cd ..
   catkin_make
   source ./devel/setup.bash
   ```

3. Connect your your kobuki base, Asus Xtion Pro, controller and usb camera.

4. Power up the kobuki base and put it on the start position. <strong>Make sure the robot is facing starting forward. </strong> 

5. Start the library

   ```bash
   roslaunch comp3 comp3.launch
   ```
   
6. View the image from usb_cam to make sure that the the white lines are at the center of the image. (e.g. You could use rqt for this task) 

6. Start the turtlebot by pressing A on the controller


## 4. Concepts & Code

### Overview

state machine:

  * Overview:
      > Note1: There are 4 phases and a wait start in the state machine. Whenver the robot saw a long red strip on the road, it will do a temperary stop.
    
      > Note2: The waite state is the starting state. When `button A` is pressed, it will enter Phase 1. And if `button B` is pressed during any phase, it will return to wait state.
  
    ![statemachine](https://github.com/CMPUT412W19Team6/Comp3/blob/master/general.PNG?s=200)
   

    
    
   * Phase1:
   
     ![statemachine](https://github.com/CMPUT412W19Team6/Comp3/blob/master/phase1.PNG?s=200)
   
   * Phase2:
   
     ![statemachine](https://github.com/CMPUT412W19Team6/Comp3/blob/master/phase2.PNG?s=200)
   
   * Phase4:
     > Note1: This phase is really long, and is generated based on an `orderedDictionary`, whose values are lists of "moving actions"
     (e.g. translation and rotation)
     
     > Note2: The last item in the `orderedDictionary` is to move robot to the exit of the parking area, i.e. the off ramp at the end
   
     ![statemachine](https://github.com/CMPUT412W19Team6/Comp3/blob/master/Phase4.PNG?s=200)
   
   * Phase3:
   
     ![statemachine](https://github.com/CMPUT412W19Team6/Comp3/blob/master/phase3.PNG?s=200)
   

### Counting objects

_Concept_:

    1. Filter out the image based on HSV (Get red color for location 1 and 3, anything but white for location 2).

    2. Count the number of contours for what's left.


### Recognizing object shapes

_Concept_:

    1. Filter out the image based on HSV. (Get green color for location 2 and red color for location 3)

    2. Found the contour with largest area.

    3. Try to fit a polygon to the contour.

    4. Determine the shape
    
        4.1 If the number of conner is 3, it's triangle.
        4.1 If the number of conner is 4, it's square.
        4.1 If the number of conner is greater than 4, it's circle.

### Following the white line

_Concept_:

    1. Filter out all the colors of the video stream except white and convert the images to be binary.

    2. Keep 20 rows of the images from the video stream starting at 3/4th the height of the images and discard all the other rows

    3. Calculate the moments and find the center of the white line. Since the images from the video have been converted to be binary, the white pixels of the white line can be considered mass whose zeroth moment represents the total mass and the first moment divided by the zeroth moment (total mass) is the center of the white line (center of mass).

    4. Adjust robot's angular velocity so that if the centroid is not at center, change robot's angular velocity to turn towards the centroid.
    
### Detecting red spots and differentiating between large and small spots

_Concept_:

    1. Filter out all the colors of the video stream except red and convert the images to be binary.

    2. Keep bottom 50% of rows of the images from the video stream.

    3. Calculate the the sum of the area of all the contours.

    4. Differentiate between large and small spots using a threshold on the sum of area.

### Going off ramp

_Concept_:

    1. After completing everything from location 2, get back on the main track and keep following it.
    2. When the first red marker is seen:
        2.1 Stop for a little while
        2.2 Go forward a little
        2.3 Turn towards the off ramp white line
        2.4 Follow the white line that is visible in the middle to its end

### Completing tasks at location 4

_Concept_:

    1. For each parking spot, do the following:
        1.1 Drive outside the box of the parking spot
        1.2 Turn to face the box
        1.3 If the spot is the designated spot for parking or there is an AR tag or an object with shape matching with the green shape of location 2
            1.3.1 Go forward until completely inside the box
            1.3.2 Do appropriate LED indication
            1.3.3 Go back until the robot is back at where it was at step 1.3
    2. Go to the exit point and switch to next phase
        

## 6.Competition Video: [Link](https://youtu.be/xGVi-B6u3OI)
