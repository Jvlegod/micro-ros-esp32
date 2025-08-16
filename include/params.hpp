#pragma once

// #define USE_ULTRASONIC
// #define USE_MPU6050
// #define USE_DIFF_DRIVE
#define USE_MICRO_AGENT

// static const char* WIFI_SSID  = "Jvle";
// static const char* WIFI_PASS  = "mkk611103";
// static IPAddress   AGENT_IP(192,168,43,172);
// static const uint16_t AGENT_PORT = 8888;

#define STM32_UART_BAUD     9600
#define STM32_UART_RX_PIN   18
#define STM32_UART_TX_PIN   17

static const char* WIFI_SSID  = "CMCC-2QDh";
static const char* WIFI_PASS  = "csbs5759";
static IPAddress   AGENT_IP(192,168,10,4);
static const uint16_t AGENT_PORT = 8888;

#define STM32_UART_BAUD   115200
#define STM32_UART_RX_PIN 18
#define STM32_UART_TX_PIN 17