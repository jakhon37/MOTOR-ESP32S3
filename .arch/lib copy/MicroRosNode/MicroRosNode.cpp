#include "MicroRosNode.h"
#include <micro_ros_platformio.h>
#include <Arduino.h> // for delay, etc.

// For config
#include <config.h>  
// For pin usage, optional LED indicator, etc.
#include <pins.h>

#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1;  \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

MicroRosNode* MicroRosNode::_instance = nullptr;

// -------------------------------------------------------------
// Constructor
// -------------------------------------------------------------
MicroRosNode::MicroRosNode(MotorWithEncoder &m1, MotorWithEncoder &m2)
: _motor1(m1), _motor2(m2), _state(WAITING_AGENT)
{
    _instance = this;  // store static reference
}

// -------------------------------------------------------------
// Setup ROS Entities
// -------------------------------------------------------------
void MicroRosNode::setupROS() {
    _allocator = rcl_get_default_allocator();
    _init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&_init_options, _allocator);

    rcl_init_options_set_domain_id(&_init_options, ROS_DOMAIN_ID);
    rclc_support_init_with_options(&_support, 0, NULL, &_init_options, &_allocator);

    rclc_node_init_default(&_node, ROS_NODE_NAME, ROS_NAMESPACE, &_support);

    // Encoder publisher
    rclc_publisher_init(
        &_encoder_pub,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/motors/encoders",
        &rmw_qos_profile_default
    );

    // Speed subscriber
    rclc_subscription_init(
        &_speed_sub,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/motors/speed_cmd",
        &rmw_qos_profile_default
    );

    // Prepare message memory (encoder has 2 data points)
    static int16_t enc_data[2] = {0, 0};
    _pub_encoders_msg.data.data = enc_data;
    _pub_encoders_msg.data.size = 2;
    _pub_encoders_msg.data.capacity = 2;

    // Timer
    const unsigned int timer_period_ms = 50; // 20 Hz
    rclc_timer_init_default(
        &_timer,
        &_support,
        RCL_MS_TO_NS(timer_period_ms),
        timerCallback
    );

    // Executor
    rclc_executor_init(&_executor, &_support.context, 2, &_allocator);
    rclc_executor_add_subscription(&_executor, &_speed_sub, &_sub_speed_msg, &speedCallback, ON_NEW_DATA);
    rclc_executor_add_timer(&_executor, &_timer);
}

// -------------------------------------------------------------
// createEntities
// -------------------------------------------------------------
bool MicroRosNode::createEntities() {
    setupROS();
    return true;
}

// -------------------------------------------------------------
// begin
// -------------------------------------------------------------
void MicroRosNode::begin() {
    // Additional initializations if needed
}

// -------------------------------------------------------------
// spinOnce
// Reconnection state machine logic
// -------------------------------------------------------------
void MicroRosNode::spinOnce() {
    switch (_state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                _state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;
        case AGENT_AVAILABLE:
            if (createEntities()) {
                _state = AGENT_CONNECTED;
            } else {
                destroy();
                _state = WAITING_AGENT;
            }
            break;
        case AGENT_CONNECTED:
            // Periodic check for agent availability
            EXECUTE_EVERY_N_MS(500,
                _state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (_state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy();
            _state = WAITING_AGENT;
            break;
        default:
            break;
    }

    // Example: set a status LED if connected
#ifdef STATUS_LED
    digitalWrite(STATUS_LED, (_state == AGENT_CONNECTED) ? HIGH : LOW);
#endif
}

// -------------------------------------------------------------
// destroy
// Clean up all micro-ROS entities
// -------------------------------------------------------------
void MicroRosNode::destroy() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&_support.context);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_timer_fini(&_timer);
    rclc_executor_fini(&_executor);
    rcl_node_fini(&_node);
    rclc_support_fini(&_support);
    rcl_init_options_fini(&_init_options);

    rcl_publisher_fini(&_encoder_pub, &_node);
    rcl_subscription_fini(&_speed_sub, &_node);
}

// -------------------------------------------------------------
// speedCallback
// -------------------------------------------------------------
void MicroRosNode::speedCallback(const void* msgin) {
    if (msgin == nullptr) return;
    const std_msgs__msg__Int16* msg = (const std_msgs__msg__Int16*) msgin;

    // Set the same speed on both motors, as an example
    if (_instance) {
        _instance->_motor1.setSpeed(msg->data);
        _instance->_motor2.setSpeed(msg->data);
    }
}

// -------------------------------------------------------------
// timerCallback
// -------------------------------------------------------------
void MicroRosNode::timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == nullptr) return;

    // Publish encoder data
    publishEncoders();
}

// -------------------------------------------------------------
// publishEncoders
// -------------------------------------------------------------
void MicroRosNode::publishEncoders() {
    if (!_instance) return;

    long enc1 = _instance->_motor1.getEncoderCount();
    long enc2 = _instance->_motor2.getEncoderCount();

    _instance->_pub_encoders_msg.data.data[0] = (int16_t)enc1;
    _instance->_pub_encoders_msg.data.data[1] = (int16_t)enc2;

    rcl_publish(&_instance->_encoder_pub, &_instance->_pub_encoders_msg, NULL);
}
