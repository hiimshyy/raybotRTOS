#ifndef __MAIN_PROCESSING_H
#define __MAIN_PROCESSING_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <SimpleKalmanFilter.h>

#define motor11Revesal 22
#define motor11PWM 2
#define motor12Revesal 24
#define motor12PWM 3
#define motor2Revesal 26
#define motor2PWM 4
#define forwardSensor A0
#define backwardSensor A1
#define updownSensor A2

#define model1 1080
#define model2 20150
#define model3 430

#define MAX_PWM 255
#define SAMPLES 25

/// @brief struct info of robot
struct Robot_info
{
    int Motor1_mode = 0;
    int Motor2_mode = 0;
    int PWM_MT_1;
    int PWM_MT_2;
    int maxSpeedMovement = 150;
    int maxSpeedLift = 150;
    int motorParam = 5;
    int maxDistanceFw = 20;
    int minDistanceFw = 10;
    int minDistanceLift = 20;
    int maxDistanceLift = 40;
    float weight;
    float battery;
    float distanceFW;
    float distanceBW;
    float distanceUD;
    bool safety;
    bool door_state;
    bool taskActive = false;
    String command;
};

class MainProcess
{
public:
    MainProcess() {};
    void begin();
    void liftBox();
    void stopMT1();
    void handleMessage();
    static void sendQueue(void *pvParameters);
    static void processingDeviceTask(void *pvParameters);
    static void handleCommandTask(void *pvParameters);
    static void serialEventTask(void *pvParameters);
    static void readDistanceTask(void *pvParameters);
    

private:
    void init();
    void serialEvent(void);
    void processingDevice(int weight = 5);
    void sort(float a[], int size);
    int distance(int8_t sensorName, int16_t modelSensor);

    void handleCommand(String cmd);
    void handleData(String data);
    void handleGetData();
    void readDistance();
    float detectTarget(int maxSpeed = 150, float distance = 20, bool modelSensor = true);

    QueueHandle_t sendMessageQueue;
    QueueHandle_t reciveMessageQueue;
    QueueHandle_t serialQueue;

    String inComingMessage;
    String isCommingMsg;
    float target;
    // float forwardDistance = 0, backwardDistance = 0, downDistance = 0, upDistance = 0, updownDistance = 0;
    float motor1Speed, motor2Speed;

    TaskHandle_t serialTaskHandle = NULL;
    TaskHandle_t sendQueueTaskHandle = NULL;
    TaskHandle_t processingDeviceTaskHandle = NULL;
    TaskHandle_t handleMessageTaskHandle = NULL;
    TaskHandle_t readDistanceTaskHandle = NULL;
};

#endif