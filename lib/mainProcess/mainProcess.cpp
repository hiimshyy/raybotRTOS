#include "mainProcess.h"

Robot_info Info;
SimpleKalmanFilter filter1(1, 1, 0.01);
SimpleKalmanFilter filter2(1, 1, 0.01);
SimpleKalmanFilter filter3(1, 1, 0.01);

void MainProcess::begin()
{
	init();
	Serial.println("[MainProcess] - Begin");

	xTaskCreate(
		MainProcess::readDistanceTask,
		"ReadDistance",
		configMINIMAL_STACK_SIZE * 4,
		this,
		tskIDLE_PRIORITY + 3,
		&readDistanceTaskHandle);

	xTaskCreate(
		MainProcess::serialEventTask,
		"SerialEvent",
		configMINIMAL_STACK_SIZE * 2,
		this,
		tskIDLE_PRIORITY + 2,
		&serialTaskHandle);

	// xTaskCreate(
	// 	MainProcess::sendQueue,
	// 	"SendQueue",
	// 	configMINIMAL_STACK_SIZE * 2,
	// 	this,
	// 	tskIDLE_PRIORITY + 2,
	// 	&sendQueueTaskHandle);

	xTaskCreate(
		MainProcess::handleCommandTask,
		"HandleMessage",
		configMINIMAL_STACK_SIZE * 4,
		this,
		tskIDLE_PRIORITY + 1,
		&handleMessageTaskHandle);

	xTaskCreate(
		MainProcess::processingDeviceTask,
		"MainProcess",
		configMINIMAL_STACK_SIZE * 4,
		this,
		tskIDLE_PRIORITY + 1,
		&processingDeviceTaskHandle);

	vTaskStartScheduler();
}

// void MainProcess::sendQueue(void *pvParameters)
// {
// 	MainProcess *self = static_cast<MainProcess *>(pvParameters);
// 	Serial.println("[MainProcess] - SendQueue");
// 	String messageToSend;
// 	for (;;)
// 	{
// 		if (xQueueReceive(self->sendMessageQueue, &messageToSend, portMAX_DELAY) == pdTRUE)
// 		{
// 			Serial.println(messageToSend);
// 			vTaskDelay(pdMS_TO_TICKS(10));
// 		}
// 	}
// }

void MainProcess::processingDeviceTask(void *pvParameters)
{
	MainProcess *self = static_cast<MainProcess *>(pvParameters);

	for (;;)
	{
		self->processingDevice();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void MainProcess::handleCommandTask(void *pvParameters)
{
	MainProcess *self = static_cast<MainProcess *>(pvParameters);
	for (;;)
	{
		if (Info.taskActive)
			self->handleCommand(Info.command);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void MainProcess::serialEventTask(void *pvParameters)
{
	MainProcess *self = static_cast<MainProcess *>(pvParameters);
	for (;;)
	{
		self->serialEvent();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void MainProcess::readDistanceTask(void *pvParameters)
{
	MainProcess *self = static_cast<MainProcess *>(pvParameters);
	self->readDistance();
}

void MainProcess::init()
{
	analogReadResolution(10);
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial.println("[MainProcess] - Init");

	const int inputPins[] = {SENSOR_FORWARD, SENSOR_BACKWARD, SENSOR_UPDOWN};
	const int outputPins[] = {MOTOR1_REV1, MOTOR1_REV2, MOTOR2_REV, MOTOR1_PWM1, MOTOR1_PWM2, MOTOR2_PWM};

	for (int pin : inputPins)
		pinMode(pin, INPUT);
	for (int pin : outputPins)
	{
		pinMode(pin, OUTPUT);
		analogWrite(pin, 0);
	}

	sendMessageQueue = xQueueCreate(100, sizeof(String));
}

void MainProcess::handleMessage()
{
	if (isCommingMsg.startsWith("CMD:"))
	{
		Info.command = isCommingMsg.substring(4);
		Serial.println("DATA:{command:" + Info.command + "}");
	}
	else if (isCommingMsg.startsWith("DATA:"))
	{
		handleData(isCommingMsg);
		isCommingMsg.remove(0);
	}
}

void MainProcess::handleCommand(const String& cmd)
{	
	// CMD:forward, CMD:backward, CMD:stop, CMD:lift_box, CMD:drop_box, CMD:clr
	if (cmd == "forward" || cmd == "backward")
	{
		// Serial.println("[handleCommand] - " + cmd);
		uint8_t newMode = (cmd == "forward") ? 1 : 2;
		if( Info.Motor1_mode != newMode)
		{
			gradualStop(MOTOR1);
			Info.Motor1_mode = newMode;
		}
		return;
	}

	if (cmd == "lift_box" || cmd == "drop_box")
	{
		uint8_t newMode = (cmd == "lift_box") ? 1 : 2;
		if(Info.Motor2_mode != newMode)
		{
			gradualStop(MOTOR2);
			Info.Motor2_mode = newMode;
		}
		return;
	}

	if (cmd == "stop")
	{
		// Serial.println("[handelCommand] - stop");
		Info.Motor1_mode = 0;
		Info.Motor2_mode = 0;
	}
	
	if (cmd == "clr")
	{
		// Serial.println("[handleCommand] - clear");
		Info.Motor1_mode = 0;
		Info.Motor2_mode = 0;
		Info.taskActive = false;
		isCommingMsg.remove(0);
		inComingMessage.remove(0);
		return;
	}
	else
	{
		Serial.println("[handleCommand] - Unknown command");
	}
}

void MainProcess::handleData(const String& data)
{
	String dataStr = data.substring(5);
	// Serial.println("[handleCommand] - handleData");	
	// DATA:{"max_pwm_movement":255,"max_pwm_lift":255, "parameter_motor":5, "max_distance_move":20, "min_distance_move":10, "min_distance_lift":20, "max_distance_lift":40}
	JsonDocument doc;
	DeserializationError error = deserializeJson(doc, dataStr);

	// Test if parsing succeeds.
	if (error)
	{
		Serial.print(F("deserializeJson() failed: "));
		Serial.println(error.f_str());
		return;
	}

	if (!doc["max_pwm_movement"].isNull())
	{
		Info.maxSpeedMovement = doc["max_pwm_movement"].as<int>(); 
		Serial.println("DATA:{max_pwm_movement:" + String(Info.maxSpeedMovement)+ "}");
	}
	if (!doc["max_pwm_lift"].isNull())
	{
		Info.maxSpeedLift = doc["max_pwm_lift"].as<int>(); 
		Serial.println("DATA:{max_pwm_lift:" + String(Info.maxSpeedLift)+ "}");
	}
	if (!doc["parameter_motor"].isNull())
	{
		Info.motorParam = doc["parameter_motor"].as<int>();
		Serial.println("DATA:{parameter_motor:" + String(Info.motorParam)+ "}");
	}
	if (!doc["max_distance_move"].isNull())

	{
		Info.maxDistanceMove = doc["max_distance_move"].as<float>(); 
		Serial.println("DATA:{max_distance_move:" + String(Info.maxDistanceMove)+ "}");
	}
	if (!doc["min_distance_move"].isNull())
	{
		Info.minDistanceMove = doc["min_distance_move"].as<float>(); 
		Serial.println("DATA:{min_distance_move:" + String(Info.minDistanceMove)+ "}");
	}
	if (!doc["min_distance_lift"].isNull())
	{
		Info.minDistanceLift = doc["min_distance_lift"].as<float>(); 
		Serial.println("DATA:{min_distance_lift:" + String(Info.minDistanceLift)+ "}");
	}
	if (!doc["max_distance_lift"].isNull())
	{
		Info.maxDistanceLift = doc["max_distance_lift"].as<float>(); 
		Serial.println("DATA:{max_distance_lift:" + String(Info.maxDistanceLift)+ "}");
	}

}

void MainProcess::processingDevice()
{
	motor1Control();
	motor2Control();
}
void MainProcess::liftBox()
{
	if (Info.distanceUD > Info.minDistanceLift)
	{
		Serial.println("[Device processing] - Lifting box before moving.");
		Info.Motor2_mode = 1;
	}
}

void MainProcess::stopMT1()
{
	if (Info.Motor1_mode != 0)
	{
		Serial.println("[Device processing] - Stopping motor 1 before lifting");

		while (Info.PWM_MT_1 > 0)
		{
			Info.PWM_MT_1 = (Info.PWM_MT_1 > 0) ? Info.PWM_MT_1 - Info.motorParam : 0;
			analogWrite(MOTOR1_PWM1, Info.PWM_MT_1);
			analogWrite(MOTOR1_PWM2, Info.PWM_MT_1);
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		Info.Motor1_mode = 0;
	}
}

void MainProcess::gradualStop(int motor){
	if(motor == 1){
		while(Info.PWM_MT_1 > 0){
			Info.PWM_MT_1 = (Info.PWM_MT_1 > 0) ? Info.PWM_MT_1 - Info.motorParam : 0;
			analogWrite(MOTOR1_PWM1, Info.PWM_MT_1);
			analogWrite(MOTOR1_PWM2, Info.PWM_MT_1);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
	else if(motor == 2){
		while(Info.PWM_MT_2 > 0){
			Info.PWM_MT_2 = (Info.PWM_MT_2 > 0) ? Info.PWM_MT_2 - Info.motorParam : 0;
			analogWrite(MOTOR2_PWM, Info.PWM_MT_2);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

void MainProcess::motor1Control(){
	if (Info.Motor1_mode == 1 || Info.Motor1_mode == 2){
		if(Info.distanceUD > Info.minDistanceLift)
			liftBox();
		else
		{
			bool isForward = (Info.Motor1_mode == 1);
			motorDriection(MOTOR1, isForward);

			float currentDistance = isForward ? Info.distanceFW : Info.distanceBW;
			motor1Speed = detectTarget(1, Info.maxSpeedMovement, currentDistance);

			updateSpeed(MOTOR1, motor1Speed, false);

			analogWrite(MOTOR1_PWM1, Info.PWM_MT_1);
			analogWrite(MOTOR1_PWM2, Info.PWM_MT_1);
			Serial.println("[Device processing] - Motor 1 speed: " + String(Info.PWM_MT_1));
		}
	}
	else 
	{
		Serial.println("[Device processing] - stop motor 1");
		updateSpeed(MOTOR1, 0, true);
		analogWrite(MOTOR1_PWM1, Info.PWM_MT_1);
		analogWrite(MOTOR1_PWM2, Info.PWM_MT_1);
	}
}

void MainProcess::motor2Control(){
	if (Info.Motor2_mode == 1 || Info.Motor2_mode == 2){
		stopMT1();

		bool isLifting = (Info.Motor2_mode == 1);
		motorDriection(MOTOR2, isLifting);

		if ((isLifting && Info.distanceUD < Info.minDistanceLift) || (!isLifting && Info.distanceUD > Info.maxDistanceLift))
		{
			motor2Speed = 0;
			Info.Motor2_mode = 0;
			Info.PWM_MT_2 = 0;
		}
		else
		{
			motor2Speed = detectTarget(2, Info.maxSpeedLift, Info.distanceUD, isLifting);
			updateSpeed(MOTOR2, motor2Speed, false);
		}
		analogWrite(MOTOR2_PWM, Info.PWM_MT_2);
		Serial.println("[Device processing] - Motor 2 speed: " + String(Info.PWM_MT_2));
	}
	else 
	{
		Serial.println("[Device processing] - stop motor 2");
		updateSpeed(MOTOR2, 0, true);
		analogWrite(MOTOR2_PWM, Info.PWM_MT_2);
	}

}

void MainProcess::updateSpeed(int motor,float target, bool isStop){
	if (motor == 1){
		if (isStop || target < MIN_PWM)
			Info.PWM_MT_1 = (Info.PWM_MT_1 > 0) ? Info.PWM_MT_1 - Info.motorParam*10 : 0;
		else
			if (target > Info.PWM_MT_1)
				Info.PWM_MT_1 += Info.motorParam;
			else if (target < Info.PWM_MT_1)
				Info.PWM_MT_1 = (Info.PWM_MT_1 > 0) ? Info.PWM_MT_1 - Info.motorParam : 0;	
	}
	else if (motor == 2){
		if (isStop || target < MIN_PWM)
			Info.PWM_MT_2 = (Info.PWM_MT_2 > 0) ? Info.PWM_MT_2 - Info.motorParam*10 : 0;
		else
			if (target > Info.PWM_MT_2)
				Info.PWM_MT_2 += Info.motorParam;
			else if (target < Info.PWM_MT_2)
				Info.PWM_MT_2 = (Info.PWM_MT_2 > 0) ? Info.PWM_MT_2 - Info.motorParam : 0;	
	}
}

void MainProcess::motorDriection(int motor, bool direction){
	if (motor == 1){
		digitalWrite(MOTOR1_REV1, direction ? LOW : HIGH);
		digitalWrite(MOTOR1_REV2, direction ? LOW : HIGH);
	}
	else if (motor == 2){
		digitalWrite(MOTOR2_REV, direction ? LOW : HIGH);
	}
}

float MainProcess::detectTarget(int motor, int maxSpeed, float distance, bool isLifting)
{
	if(motor == 1){
		if (distance > Info.maxDistanceMove)
			target = maxSpeed;
		else if (distance < Info.maxDistanceMove && distance > Info.minDistanceMove)
			target = max(0,(distance * maxSpeed) / Info.maxDistanceMove);
		else
			target = 0;
	}
	else if (motor == 2)
	    if(isLifting)
			if (distance > Info.maxDistanceLift)
				target = maxSpeed;
			else if (distance < Info.maxDistanceLift && distance > Info.minDistanceLift)
				target = max(0,(distance * maxSpeed) / Info.maxDistanceLift);
			else
				target = 0;
		else
			if (distance < Info.minDistanceLift)
				target = maxSpeed;
			else if (distance > Info.minDistanceLift && distance < Info.maxDistanceLift)
				target = max(0,(distance * maxSpeed) / Info.maxDistanceLift);
			else
				target = 0;
	else
		target = 0;
	return max(0,target);
}

void MainProcess::readDistance()
{
	for (;;)
	{
		Info.distanceFW = filter1.updateEstimate(distance(SENSOR_FORWARD, MODEL_1080));
		Info.distanceBW = filter2.updateEstimate(distance(SENSOR_BACKWARD, MODEL_1080));
		Info.distanceUD = filter3.updateEstimate(distance(SENSOR_UPDOWN, MODEL_20150));

		// JsonDocument doc;
		// // doc["forward_distance"] = int(Info.distanceFW);
		// // doc["backward_distance"] = int(Info.distanceBW);
		// // doc["lift_distance"] = int(Info.distanceUD);

		// String buffer;
		// serializeJson(doc, buffer);
		String message = "DATA:{\"forward_distance\":" + String(int(Info.distanceFW)) +
						 ",\"backward_distance\":" + String(int(Info.distanceBW)) +
						 ",\"lift_distance\":" + String(int(Info.distanceUD)) + "}";

		// In chuỗi JSON ra Serial
		Serial.println(message);

		// if (sendMessageQueue != nullptr)
		// {
		// 	xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(10));
		// }

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void MainProcess::sort(float a[], int size)
{
	for (int i = 0; i < size - 1; i++)
	{
		bool swapped = false;
		for (int j = 0; j < size - i - 1; j++)
		{
			if (a[j] > a[j + 1])
			{
				float temp = a[j];
				a[j] = a[j + 1];
				a[j + 1] = temp;
				swapped = true;
			}
		}
		if (!swapped)
			break;
	}
}

int MainProcess::distance(int8_t sensorName, int16_t modelSensor)
{
	float ir_val[SAMPLES] = {};
	int distanceCM;
	int median;

	for (int i = 0; i < SAMPLES; i++)
	{
		ir_val[i] = analogRead(sensorName);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	sort(ir_val, SAMPLES);
	median = ir_val[SAMPLES / 2];

	if (modelSensor == 430)
		if (median > 1000)
			distanceCM = 0;
		else if (median < 1000 && median > 150)
			distanceCM = 12.08 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.058);
		else 
			distanceCM =  30;
	else if (modelSensor == 1080)
		if (median > 1000)
			distanceCM = 0;
		else if (median < 1000 && median > 150)
			distanceCM = 29.988 * pow(map(median, 0, 1023, 0, 5000) / 1000.0, -1.173);
		else
			distanceCM = 80;
	else if (modelSensor == 20150)
		if (median > 1000)
			distanceCM = 0;
		else if (median < 1000 && median > 150)
			distanceCM = 60.374 * pow(map(median, 0, 1023, 0, 5000) / 1000.0, -1.16);
		else
			distanceCM = 150;
	else
		distanceCM = 0;
	return distanceCM;
}

void MainProcess::handleGetData()
{
	Serial.println("[handleCommand] - handleGetData");
	JsonDocument jsonDoc;

	jsonDoc["forward_distance"] = Info.distanceFW;
	jsonDoc["backward_distance"] = Info.distanceBW;
	jsonDoc["lift_distance"] = Info.distanceUD;
	jsonDoc["Info.motorParam"] = Info.weight;
	jsonDoc["battery"] = Info.battery;
	jsonDoc["movement_motor"] = Info.Motor1_mode;
	jsonDoc["movement_pwm"] = Info.PWM_MT_1;
	jsonDoc["lift_motor"] = Info.Motor2_mode;
	jsonDoc["lift_pwm"] = Info.PWM_MT_2;
	jsonDoc["safety"] = Info.safety;
	jsonDoc["door_state"] = Info.door_state;

	String buffer;
	serializeJson(jsonDoc, buffer);
	String message = "DATA:" + buffer;
	Serial.println(message);
	if (sendMessageQueue != nullptr)
	{
		xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(10));
	}
}

void MainProcess::serialEvent(void)
{
	if (Serial.available())
	{
		inComingMessage = Serial.readStringUntil('\n');
		inComingMessage.trim();
		if (inComingMessage.length() > 0)
		{
			Serial.println("[Serial Event] - Received:" + inComingMessage);
			isCommingMsg = inComingMessage;
			Info.taskActive = true;
			handleMessage();
			inComingMessage = "";
		}	
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}