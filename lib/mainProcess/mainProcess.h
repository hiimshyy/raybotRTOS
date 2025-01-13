#ifndef __MAIN_PROCESSING_H
#define __MAIN_PROCESSING_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <SimpleKalmanFilter.h>

#define MOTOR1_REV1     22
#define MOTOR1_PWM1     2
#define MOTOR1_REV2     24
#define MOTOR1_PWM2     3
#define MOTOR2_REV      26
#define MOTOR2_PWM      4
#define SENSOR_FORWARD  A0
#define SENSOR_BACKWARD A1
#define SENSOR_UPDOWN   A2

#define MODEL_1080      1080
#define MODEL_20150     20150
#define MODEL_430       430

#define MAX_PWM         255
#define MIN_PWM         20
#define SAMPLES         25


/// @brief struct info of robot
struct Robot_info
{
    int Motor1_mode = 0;
    int Motor2_mode = 0;
    int PWM_MT_1 = 0;
    int PWM_MT_2 = 0;
    int maxSpeedMovement = 150;
    int maxSpeedLift = 150;
    int motorParam = 5;
    int maxDistanceMove = 20;
    int minDistanceMove = 10;
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

class MainProcess{
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
    enum Motor{
        MOTOR1 = 1,
        MOTOR2 = 2
    };
    void init();
    void serialEvent(void);
    void processingDevice();
    void sort(float a[], int size);
    int distance(int8_t sensorName, int16_t modelSensor);

    void handleCommand(const String& cmd);
    void handleData(const String& data);
    void handleGetData();
    void readDistance();
    void gradualStop(int motor);
    void motor1Control();
    void motor2Control();
    void motorDriection(int motor, bool direction);
    void updateSpeed(int motor, float target, bool isStop = false);
    float detectTarget(int motor, int maxSpeed, float distance, bool isLifting = false);

    QueueHandle_t sendMessageQueue;
    QueueHandle_t reciveMessageQueue;

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

#endif // __MAIN_PROCESSING_H