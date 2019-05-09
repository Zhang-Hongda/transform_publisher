transform_publisher
==============================
****
|Author|Zhang Hongda|
|------|-----------------|
|E-mail|2210010444@qq.com|
|Institute|Harbin Institute of Technology|
****

### Description
A tf broadcaster with a graphical user interface.
### Installation
Step 1: clone the repository into your own workspace
```
cd ${PATH_TO YOUR_WORKSPACE_FOLDER}/src
git clone https://github.com/Zhang-Hongda/transform_publisher
```
Step 2: building
```
catkin_make
```
Step 3: activate the workspace
```
source ${PATH_TO YOUR_WORKSPACE_FOLDER}/devel/setup.bash
```
### Strat 
To start the program, run:
```
roscore
rosrun transform_publisher transform_publisher
```
### GUI
The interface should look like this:
![tf_publisher.png](./png/tf_publisher.png "tf publisher")
### Usage
* The "__Ros Master__" panel
    * Users can specify the __url__ and __ip__ for connecting the ros master.
    * Check "__Use environment variables__" if you want to use the environmental configuration of your computer (recommended).
    * Check "__Remember settings on stratup__" to allows the system to load the current configuration during startup next time.
* The "__tf publisher__" panel
    * Users can specify the __frame_id__ and __child frame_id__ for the messages publisher.
    * Modify the values in __x__,__y__,__z__,__row__,__pitch__,__yaw__ to change the tf messages.
    * Click on "__publish__" button to publish tf messages.

