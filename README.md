# ROS_OpenCV_Pythonlib
  Contains a Python library with useful functions intregrating **OpenCV** computer vision package (*opencv-python*) and **ROS** client package (*rospy*).

  This repository is was developed to be a python libraries package for other projects, using ROS and/or OpenCV, therefore it's likely that the need of place it within another repository arises. Unfortunately though simply using  <code>git clone <url_repo></code>  isn't enough to prevent error rising.

  Luckily there's a dedicated git service which deals with this need: <code>submodules</code>. The simplest usage is described below but in order to fully understand how it works take a look at the [official guide](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

  To add it inside your project's repository run this command:

<code>git clone --recurse-submodule https://github.com/DualStream799/ROS_OpenCV_Pythonlib.git</code>

  After that is required to import the desired python libraries. On your own *.py* file add the following lines bellow all other imports:

>  <code>import sys</code>
>
> <code>sys.path.append('../ROS_OpenCV_Pythonlib') #insert path to the "ROS_OpenCV_Pythonlib" folder</code> 
>
> <code>from bot_module import *</code>

  Be sure that the path was correctly specified.

  There are 3 classes inside the *bot_module* file:

* ***ControlBotModule***: contains commands to properly control and to deal with data obtained by sensors using *rospy*. Can be used separately.
* ***VisionBotModule***:  contains computer vision algorithms and frames' dealers using *opencv-python*. Can be used separately.
* ***SupportBotModule***: contains secondary algorithms or calculations used in specific methods on the previous two classes. Direct importation is not required.