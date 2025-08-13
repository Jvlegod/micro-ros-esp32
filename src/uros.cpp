#include "uros.hpp"

#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>
#include "diff_drive.hpp"

extern DiffDrive diff_drive;

namespace uros {

static InitConfig s_cfg;
static Handles s_out;

// subscribe
static SubscriberHandles_t uros_cmd_vel;
static PublisherHandles_t uros_odom;

static void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    Serial.println("[micro-ROS] cmd_vel received:");
    float v_x = msg->linear.x;
    float w_z = msg->angular.z;
    Serial.printf("- %f %f\n", v_x, w_z);

    diff_drive.setTargetBodySpeed(v_x, w_z);
}

static void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {
    Odom_t odom = diff_drive.odom();
    float half = 0.5f * odom.yaw;
    int64_t stamp = rmw_uros_epoch_millis();
    float s, c; sincosf(half, &s, &c);

    uros_odom.msg.header.stamp.sec = stamp / 1000;
    uros_odom.msg.header.stamp.nanosec = (stamp % 1000) * 1000000;
    uros_odom.msg.pose.pose.position.x = odom.x;
    uros_odom.msg.pose.pose.position.y = odom.y;
    
    uros_odom.msg.pose.pose.orientation.w = c;
    uros_odom.msg.pose.pose.orientation.x = 0.0;
    uros_odom.msg.pose.pose.orientation.y = 0.0;
    uros_odom.msg.pose.pose.orientation.z = s;
    uros_odom.msg.twist.twist.linear.x = diff_drive.vLinear();
    uros_odom.msg.twist.twist.angular.z = diff_drive.vAngular();

    if (rcl_publish(&uros_odom.pub, &uros_odom.msg, NULL) != RCL_RET_OK) {
        Serial.println("[micro-ROS] Failed to publish odom message");
        return;
    }
}

void configure(const InitConfig& cfg) {
  s_cfg = cfg;
}

void uros_task_entry(void*) {
    Serial.println("[micro-ROS] Config WiFi UDP transport ...");
    set_microros_wifi_transports((char*)s_cfg.ssid, (char*)s_cfg.pass, s_cfg.agent_ip, s_cfg.agent_port);
    delay(2000);

    s_out.allocator = rcl_get_default_allocator();

    rcl_ret_t rc = rclc_support_init(&s_out.support, 0, NULL, &s_out.allocator);
    if (rc != RCL_RET_OK) {
      Serial.printf("[micro-ROS] rclc_support_init failed: %d\n", rc);
      return;
    }

    rc = rclc_node_init_default(&s_out.node, s_cfg.node_name, s_cfg.ns, &s_out.support);
    if (rc != RCL_RET_OK) {
      Serial.printf("[micro-ROS] rclc_node_init_default failed: %d\n", rc);
      return;
    }

    rc = rclc_executor_init(&s_out.executor, &s_out.support.context,
                            s_cfg.executor_handles, &s_out.allocator);
    if (rc != RCL_RET_OK) {
      Serial.printf("[micro-ROS] rclc_executor_init failed: %d\n", rc);
      return;
    }

    // add subscriber
    rclc_subscription_init_best_effort(
      &uros_cmd_vel.sub,
      &s_out.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");
    rclc_executor_add_subscription(&s_out.executor, &uros_cmd_vel.sub, &uros_cmd_vel.msg, &cmd_vel_callback, ON_NEW_DATA);
    
    // add publisher
    uros_odom.msg.header.frame_id.data = (char*)"odom";
    uros_odom.msg.child_frame_id.data = (char*)"base_footprint";
    rclc_publisher_init_best_effort(&uros_odom.pub,
                                    &s_out.node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                    "/odom");
    rclc_timer_init_default(&uros_odom.timer, &s_out.support, s_cfg.ping_ms, odom_callback);
    while (rmw_uros_epoch_synchronized() == false) {
        Serial.println("[micro-ROS] Waiting for epoch synchronization...");
        rmw_uros_sync_session(1000);
        delay(10);
    }

    rclc_executor_add_timer(&s_out.executor, &uros_odom.timer);

    Serial.println("[micro-ROS] Connected. Node & Executor ready.");

    rclc_executor_spin(&s_out.executor);
}

} // namespace uros