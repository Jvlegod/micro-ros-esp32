#pragma once
#include <Arduino.h>
#include <IPAddress.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

namespace uros {

struct Handles {
    rcl_allocator_t   allocator{};
    rclc_support_t    support{};
    rcl_node_t        node{};
    rclc_executor_t   executor{};
};

typedef struct subscriber_handles {
    rcl_subscription_t sub;
    geometry_msgs__msg__Twist msg;
} SubscriberHandles_t;

typedef struct publisher_handles {
    rcl_publisher_t pub;
    rcl_timer_t timer;
    nav_msgs__msg__Odometry msg;
} PublisherHandles_t;

struct InitConfig {
    const char* ssid;
    const char* pass;
    IPAddress   agent_ip;
    uint16_t    agent_port;
    const char* node_name = "robot_esp32";
    const char* ns = "";
    uint32_t    ping_ms = RCL_MS_TO_NS(50);
    int         attempts = 5;
    int         executor_handles = 8;
};

void configure(const InitConfig& cfg);

void uros_task_entry(void*);

} // namespace uros
