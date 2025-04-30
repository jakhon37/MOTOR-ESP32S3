#pragma once

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>

#include <MotorWithEncoder.h>

class MicroRosNode
{
public:
    MicroRosNode(MotorWithEncoder &m1, MotorWithEncoder &m2);
    void begin();
    void spinOnce();
    void destroy();

private:
    // Motor references
    MotorWithEncoder& _motor1;
    MotorWithEncoder& _motor2;

    // micro-ROS core
    rcl_node_t        _node;
    rcl_timer_t       _timer;
    rclc_support_t    _support;
    rcl_init_options_t _init_options;
    rclc_executor_t   _executor;
    rcl_allocator_t   _allocator;

    // Publishers / Subscribers
    rcl_publisher_t    _encoder_pub;
    rcl_subscription_t _speed_sub;

    // Messages
    std_msgs__msg__Int16            _sub_speed_msg;
    std_msgs__msg__Int16MultiArray  _pub_encoders_msg;

    // Reconnection state
    typedef enum {
      WAITING_AGENT,
      AGENT_AVAILABLE,
      AGENT_CONNECTED,
      AGENT_DISCONNECTED
    } AgentState;
    AgentState _state;

    static void speedCallback(const void* msgin);
    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);

    // Helper
    bool createEntities();
    void setupROS();
    static void publishEncoders();

    // Let static callbacks access instance
    static MicroRosNode* _instance;
};
