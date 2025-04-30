
// include/config.h

#pragma once

// Some global application config:

#ifndef ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 0
#endif

// Default micro-ROS node name and namespace:
#define ROS_NODE_NAME "esp32_c3_motor_node"
#define ROS_NAMESPACE ""

// WiFi credentials and agent info (set from platformio.ini if USE_WIFI is defined):
#ifdef USE_WIFI
  #ifndef WIFI_SSID
    #define WIFI_SSID "U+NetEFDA"
  #endif
  #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD "5D2A1A9EM#"
  #endif
  #ifndef AGENT_IP
    #define AGENT_IP "192.168.219.107"
  #endif
  #ifndef AGENT_PORT
    #define AGENT_PORT 8888
  #endif
#endif

