#ifndef __MAIN_PROCESSING_H
#define __MAIN_PROCESSING_H

#include <Arduino.h>
#include <ArduinoJson.h>  
#include <FreeRTOS.h>
#include <queue.h>

#define _revesal_motor_1 	22 	
#define _pwm_motor_1  		2 	
#define _revesal_motor_2 	24  
#define _pwm_motor_2  		3  
#define _forward_distance_sensor 	A0		
#define _backward_distance_sensor	A1	
#define _up_down_distance_sensor 	A2

/// @brief struct info of robot
struct Robot_info {
	String Name = "robot_transporter";
	int Stage = 0;
	int Motor1_mode = 0;
	int Motor2_mode = 0;
	int PWM_MT_1;
	int PWM_MT_2;
	float weight;
	float battery;
	float distanceFW;
	float distanceBW;
    float distanceUD;
	bool safety;
	bool door_state;
};

class MainProcess {
    public:
        MainProcess() {};
        void begin();
        void liftBox();
        void stopMT1();
        void handleMessage();
        static void sendQueue(void* pvParameters);
        static void processingDeviceTask(void* pvParameters);
        static void handleMessageTask(void* pvParameters);
        static void serialEventTask(void* pvParameters);
        static void readDistanceTask(void* pvParameters);
        bool stringComplete = false;
        String isCommingMsg;

    private:
        void init();
        void serialEvent(void);
        void processingDevice(int weights);
        
        void handleCommand(String& command);
        void handleData(String& data);
        void handleGetData();
        void readDistance();

        
        QueueHandle_t sendMessageQueue;
        QueueHandle_t serialQueue;
        
        TaskHandle_t serialTaskHandle = NULL;
        TaskHandle_t sendQueueTaskHandle = NULL;
        TaskHandle_t processingDeviceTaskHandle = NULL;
        TaskHandle_t handleMessageTaskHandle = NULL;
        TaskHandle_t readDistanceTaskHandle = NULL;

        uint8_t motor1Stage = 0, motor2Stage = 0;

};

#endif