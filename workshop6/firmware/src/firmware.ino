// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <geometry_msgs/msg/vector3.h>

#include <std_srvs/srv/set_bool.h>

#define LED_PIN  2


// กำหนดขา RGB LED
#define RED_PIN    25
#define GREEN_PIN  26
#define BLUE_PIN   27



#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef NODE_NAME
#define NODE_NAME "esp32_service_led_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)



rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;


rcl_service_t  service_led;


std_srvs__srv__SetBool_Request req;
std_srvs__srv__SetBool_Response res;



void led_service_callback(const void * req_in, void * res_out) {
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *) req_in;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *) res_out;

  if (request->data) {
    // เชื่อมต่ออุปกรณ์ หรือทำบางอย่าง
    
    setRGBColor(true,false,false);
    response->success = true;
    response->message.data = strdup("ESP32 Turn LED ON");
    
    
  } else {
    setRGBColor(false,false,false); 
    response->success = true;
    response->message.data = strdup("ESP32 Turn LED OFF");
  }
}


enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


void setup() 
{

    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
    
    // set RGB Module pinout
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    
    setRGBColor(false,false,false);


    

    set_microros_serial_transports(Serial);

}

void loop() {

    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            
            
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    

}







bool createEntities()
{
   
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    
   
    
   
    rclc_service_init_default(&service_led, 
                                     &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),     
                                     "service_onoff_led");
    
                                                                     
                                                                                                        
    
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, & allocator));
    
    RCCHECK(rclc_executor_add_service(&executor, &service_led,   &req, &res, led_service_callback));
    
    
    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

   
       
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, LOW);
    
    return true;
}




bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
	timespec tp;
	tp.tv_sec = time_ns / 1000000000;
	tp.tv_nsec = time_ns % 1000000000;
	clock_settime(CLOCK_REALTIME, &tp);
#else
	unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	// now we can find the difference between ROS time and uC time
	time_offset = ros_time_ms - millis();
#endif
	return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
	
    }
}
void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}
void setRGBColor(bool r, bool g, bool b) {

    digitalWrite(RED_PIN, r ? HIGH : LOW);
    digitalWrite(GREEN_PIN, g ? HIGH : LOW);
    digitalWrite(BLUE_PIN, b ? HIGH : LOW);
}


