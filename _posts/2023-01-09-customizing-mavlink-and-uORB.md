---
layout: post
title: "Customizing Mavlink and uORB message for PX4 Firmware"
excerpt_separator: "<!--more-->"
categories:
  - My Research
tags:
  - Drone
  - APF
  - Artificial Potential Field
  - Reinforcement Learning
use_math: true
---

Source: [PX4 Sending a Custom Message from MAVROS to PX4](https://docs.px4.io/main/en/ros/mavros_custom_messages.html)

## Introduction

<br>
In order to make my PX4 firmware accept my custom control command comming from actor neural net, I made a framework with steps specified below: 
<br>
<br>

1. All the required inputs are put into single <b>ROS module (Drone Brain)</b> to get the output (actuator commands)

2. Those actuator commands are "packaged" in a single <b>MAVROS message</b> and converted by a <b>custom MAVROS plugin</b> into <b>Mavlink message</b>

3. This Mavlink message is sent by a means of communication (e.g. UART, Telemetry, etc)

4. PX4 firmware takes the message and converts into a <b>uORB message</b>, which will be parsed and processed by the driver module to get actuator outputs compatible with my actual drone's ESC.

<br>

Fortunately, there is a way to implement my framework after the 1st step almost exatly the same. I put the source at the beginning of my post. Feel free to check out.

<br>

 <h2> MAVROS </h2>


As instructed, I created my custom MAVROS plugin which subscribes an external ROS topic then converts to Mavlink message. However, I made a slight change for my designed framework: 

- The topic message type will be "std_msgs/Float32MultiArray" instead of "std_msgs/Char."
- The names of this plugin will be changed to something related to "neural command." 

<br>


1. <b>Below is my code for "neuralcommand.cpp", c++ source file for MAVROS plugin: </b> 

<br>

(workspace/src/mavros/mavros_extras/src/plugins/neuralcommand.cpp)


```cpp

#include <iostream>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32MultiArray.h>

namespace mavros {

namespace extra_plugins{

// My MAVROS Plugin named "NeuralCommandPlugin"
class NeuralCommandPlugin: public plugin::PluginBase {
public:
    int size = 4; // Size of my actuator output array

    // Class constructor with member initializer list
    NeuralCommandPlugin(): PluginBase(), nh("~neural_command") {};

    // Maybe related to "Make ROS publisher and subscriber" tutorial, but not sure about the purposes of Initialize(UAS &uas) and get_subscriptions() yet. Maybe related to not using "int main()" but just using this class when run by PX4 Commander module
    void initialize(UAS &uas_) 
    {
        PluginBase::initialize(uas_);

        // Subscribe to the topic /neural_sub, and raise callback function neural_cb
        neural_sub = nh.subscribe("neural_sub", 10, &NeuralCommandPlugin::neural_cb, this);

    }

    Subscriptions get_subscriptions() 
    {
        return {};
    }

private:

    // ROS node handle for subscribing topic /neural_sub
    ros::NodeHandle nh;
    ros::Subscriber neural_sub;

    // Callback function when getting ROS message with type "Float32MultiArray"
    void neural_cb(const std_msgs::Float32MultiArray::ConstPtr &req) 
    {
        for (int i = 0; i < size; i++) {
            std::cout << "Got Data : " << req->data[i] <<  std::endl;
        }
        
        // Declare my custom Mavlink message "NEURAL_COMMAND"
        mavlink::common::msg::NEURAL_COMMAND nc {};

        //nc.command[0] = req->data; //This will be uncommend when testing
        
        // For debug purpose

        float m1 = 1500;
        float m2 = 1500;
        float m3 = 1500;
        float m4 = 1500;

        nc.command[0] = m1;
        nc.command[1] = m2;
        nc.command[2] = m3;
        nc.command[3] = m4;

        // Send NEURAL_COMMAND Mavlink message
        UAS_FCU(m_uas)->send_message_ignore_drop(nc);
    }

};


}

}

// Export this class to MAVROS package
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::NeuralCommandPlugin, mavros::plugin::PluginBase)

```
<br>

2. <b>Make MAVROS Package aware of my custom plugin:</b> 

<br>

(workspace/src/mavros/mavros_extras/mavros_plugins.xml)

```xml
...
  <class name="neural_command" type="mavros::extra_plugins::NeuralCommandPlugin" base_class_type="mavros::plugin::PluginBase">
     <description>Send motor command from NN.</description>
  </class>
...

```
<br>
3. <b>Add my plugin source file in CMakeLists.txt to make "catkin build" compile and build it:</b>

<br>
(workspace/src/mavros/mavros_extras/CMakeLists.txt)

```

add_library(mavros_extras

...

  src/plugins/neuralcommand.cpp
)
```
<br>
4. <b>Define my custom Mavlink message:</b>

<br>
(workspace/src/mavlink/message_definitions/v1.0/common.xml)

```xml
...
    <message id="229" name="NEURAL_COMMAND">
     <description>Neural motor command.</description>
     <field type="float[4]" name="command"> </field>
   </message>
...
```
<b>Caution: do not put "float32" instead of float. It will cause type error in CMake</b>   
<br>

## PX4 Firmware

<br>

1. <b>Define the same new Mavlink message to make PX4 Firmware aware of it: </b>

<br>

(PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml)

```xml
...
    <message id="229" name="NEURAL_COMMAND">
    <description>Neural motor command.</description>
    <field type="float[4]" name="command"> </field>
   </message>
...
```
<b>Caution: common.xml in MAVROS Package and PX4 Firmware MUST be the same.</b>

<br>

2. <b>Make custom uORB message "neural command":</b>

<br>
(PX4-Autopilot/msg/neural_command.msg)

```
uint64 timestamp # time since system start (microseconds)
float32[4] command
```

3. <b>Include uORB message source file into CMakeLists.txt to compile: </b>

<br>
(PX4-Autopilot/msg/CMakeLists.txt)

```
set(msg_files
...
	neural_command.msg
)
...
```
<br>

4. <b>Add below codes into mavlink_receiver.h to receive my Mavlink message: </b>

<br>
(PX4-Autopilot/src/modules/mavlink/mavlink_receiver.h)

```cpp

class MavlinkReceiver : public ModuleParams
{
public:
	MavlinkReceiver(Mavlink *parent);
	~MavlinkReceiver() override;

	void start();
	void stop();

	bool component_was_seen(int system_id, int component_id);
	void enable_message_statistics() { _message_statistics_enabled = true; }
	void print_detailed_rx_stats() const;

	void request_stop() { _should_exit.store(true); }

private:
	static void *start_trampoline(void *context);
	void run();

	void acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result, uint8_t progress = 0);

	/**
	 * Common method to handle both mavlink command types. T is one of mavlink_command_int_t or mavlink_command_long_t.
	 */
	template<class T>
	void handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
					 const vehicle_command_s &vehicle_command);

	uint8_t handle_request_message_command(uint16_t message_id, float param2 = 0.0f, float param3 = 0.0f,
					       float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

	void handle_message(mavlink_message_t *msg);

...
	
    // Put this line (my mavlink message receiver)
    // Input: dereferenced data
    void handle_message_neural_command(mavlink_message_t *msg);

...
	
    orb_advert_t _neural_command_pub{nullptr};

...
```

<br>

5. <b> Modify "mavlink_receiver.cpp"file to actually handle my message: </b> 

<br>
(PX4-Autopilot/src/modules/mavlink/mavlink_receiver.cpp)

```cpp
...

// Neural Command Mavlink Message Handler
void
MavlinkReceiver::handle_message_neural_command(mavlink_message_t *msg)
{
    mavlink_neural_command_t man; // create mavlink message_t type variable
    mavlink_msg_neural_command_decode(msg, &man); // decode mavlink message

struct neural_command_s key = {}; // create uORB message variable to package data
// Maybe it can be changed to uORB topic /actuator_output to overwrite data from mc_att_controller module

    // put data in defined order:
    key.timestamp = hrt_absolute_time();
    key.command[0] = man.command[0];
	key.command[1] = man.command[1];
	key.command[2] = man.command[2];
	key.command[3] = man.command[3];
	

    // if publisher not spawned, advertise first time
    if (_neural_command_pub == nullptr) {
        _neural_command_pub = orb_advertise(ORB_ID(neural_command), &key);

    // if publisher already exists, send uORB message via this topic
    } else {
        orb_publish(ORB_ID(neural_command), _neural_command_pub, &key);
    }
}

...

```

<br>

6. <b>Create PX4 custom application (uORB topic subscriber)</b> <br> (Maybe I can modify this process to make my uORB subscriber run in "Offboard Mode" or in existing module such as mixer or driver?) <br> <br> This module source folder (/PX4-Autopilot/src/modules/command_receiver) contains the below source files:



<br>

- CMakeLists.txt

(/PX4-Autopilot/src/modules/command_receiver/CMakeLists.txt)

```

px4_add_module(
    MODULE modules__command_receiver
    MAIN command_receiver
    STACK_MAIN 2500
    STACK_MAX 4000
    SRCS
        command_receiver.cpp
    DEPENDS

    )

```

<br>

- command_receiver.cpp

(/PX4-Autopilot/src/modules/command_receiver/command_receiver.cpp)

```cpp

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <px4_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/neural_command.h>

extern "C" __EXPORT int command_receiver_main(int argc, char **argv);

int command_receiver_main(int argc, char **argv)
{
    int neural_sub_fd = orb_subscribe(ORB_ID(neural_command)); // subscribes to uORB topic /neural_command

    orb_set_interval(neural_sub_fd, 200); // limit the update rate to 200ms

    px4_pollfd_struct_t fds[] = {
        { .fd = neural_sub_fd,   .events = POLLIN },
    };

    int error_counter = 0;

    for (int i = 0; i < 10; i++) // takes message 10 times and stop
    {
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        }

        else if (poll_ret < 0)
        {
            if (error_counter < 10 || error_counter % 50 == 0)
            {
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;
        }

        else
        {
            if (fds[0].revents & POLLIN)
            {
                struct neural_command_s input;
                orb_copy(ORB_ID(neural_command), neural_sub_fd, &input);
                PX4_INFO("Received Data : %f", (double)input.command[0]);
             }
        }
    }

    return 0;
}

```

<b>Caution: the module name "command_receiver" must be the same throughout this module source folder! Otherwise, CMake will raise file reference error</b>

<br>

- KConfig

(/PX4-Autopilot/src/modules/command_receiver/KConfig)

```
 menuconfig MODULES_COMMAND_RECEIVER
 bool "command_receiver"
 default n
 ---help---
 	Enable support for key_receiver
```

<br>

7. <b> Add module in "default.px4board" file correspond to my board (e.g. fmuv5). If SITL, go to /PX4-Autopilot/boards/px4/sitl/default.px4board. </b>

<br>

(/PX4-Autopilot/boards/px4/sitl/default.px4board)


```
CONFIG_MODULES_COMMAND_RECEIVER=y
```

<br>

## Result

As instructed in the source link, I first compiled MAVROS plugin using "catkin build." I had to deal with compile errors such as invalid type (float32) error, minor syntax error, and file reference error.

<br>

<img src ="/assets/images/mavroscompileerrorpng.png" width = "" height = "" title ="MAVROS Make Error">
<figcaption align = "left"><b> [Image 1]: Invalid type compile error</b></figcaption>

<br>

<img src ="/assets/images/mavroscompilesuccess.png" width = "" height = "" title ="MAVROS Make Success">
<figcaption align = "left"><b> [Image 2]: MAVROS Successfully Compiled </b></figcaption>


<br>

After compiling it, I sourced my workspace file to linux environment (~/.bashrc) and launched MAVROS using "roslaunch mavros px4.launch."

<br>

<img src ="/assets/images/mavroslaunch.png" width = "" height = "" title ="MAVROS Launch">

<img src ="/assets/images/mavroslaunch2.png" width = "" height = "" title ="MAVROS Launch2">

<figcaption align = "left"><b> [Image 3]: MAVROS Launched Successfully </b></figcaption>

<br>

Then, I compiled PX4 Firmware and launched Gazebo SITL environment using "make px4_sitl gazebo" in PX4-Autopilot root file.

<br>

<img src ="/assets/images/px4make.png" width = "" height = "" title ="PX4 made successfully">

<img src ="/assets/images/gazebo.png" width = "" height = "" title ="PX4 gazebo">
<figcaption align = "left"><b> [Image 4]: PX4 Make Gazebo Successful </b></figcaption>

<br>

For testing, I sent ROS message with random content via /neural_sub topic

<br>

<img src ="/assets/images/rostopicpub.png" width = "" height = "" title ="ROS publish message">
<figcaption align = "left"><b> [Image 5]: Publishing ROS message </b></figcaption>

<br>

This is my result:

<br>

<img src ="/assets/images/mavrosreceiving.png" width = "" height = "" title ="ROS publish message">

<img src ="/assets/images/px4apprunning.png" width = "" height = "" title ="ROS publish message">

<figcaption align = "left"><b> [Image 6]: MAVROS and PX4 Receiving Message Successfully </b></figcaption>

<br>

MAVROS first gets /neural_sub topic and sends custom Mavlink message, which is then processed in PX4 Firmware into custom uORB message and gives the content of data.

<br>

# TO DO

1. Make ROS node getting PX4 drone orientation and check receiving rate

2. Make ROS node publishing motor commands continuously

3. Now that my PX4 Firmware receives uORB message via custom application:
    
    - Check if uORB message can be received in existing, desired module

    - Make PX4 module accept my custom motor command and run in SITL environment and check if this causes message conflicts by <i>mc_att_controller</i> and <i>control_allocator</i> modules


4. Connect above frameworks with python implementation of parsing ONNX and inferencing.

