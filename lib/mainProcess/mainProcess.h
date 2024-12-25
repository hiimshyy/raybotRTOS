#ifndef __MAIN_PROCESSING_H
#define __MAIN_PROCESSING_H

#include <Arduino.h>
#include <ArduinoJson.h>  
#include <FreeRTOS.h>
#include <queue.h>

#define motor1Revesal 	22 	
#define motor1PWM  		2 	
#define motor2Revesal 	24  
#define motor2PWM  		3  
#define forwardSensor 	A0		
#define backwardSensor	A1	
#define updownSensor 	A2

#define model1    1080
#define model2    20150

#define SAMPLES 25


/// @brief struct info of robot
struct Robot_info {
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
    bool taskActive = true;
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
        String isCommingMsg;
        float target, maxSpeed;

    private:
        void init();
        void serialEvent(void);
        void processingDevice(int weights);
        void sort(float a[], int size);
        int distance(int8_t sensorName, int16_t modelSensor);
        
        void handleCommand(String& command);
        void handleData(String& data);
        void handleGetData();
        void readDistance();
        float detectTarget(float maxSpeed, float distance);

        QueueHandle_t sendMessageQueue;
        QueueHandle_t reciveMessageQueue;
        QueueHandle_t serialQueue;
        
        TaskHandle_t serialTaskHandle = NULL;
        TaskHandle_t sendQueueTaskHandle = NULL;
        TaskHandle_t processingDeviceTaskHandle = NULL;
        TaskHandle_t handleMessageTaskHandle = NULL;
        TaskHandle_t readDistanceTaskHandle = NULL;

        uint8_t motor1Stage = 0, motor2Stage = 0;

};

#endif