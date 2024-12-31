#include "mainProcess.h"

Robot_info Info;
String inComingMessage;
float forwardDistance = 0, backwardDistance = 0, downDistance = 0, upDistance = 0, updownDistance =0;
float motor1Speed, motor2Speed;

void MainProcess::begin() {
    init();
	sendMessageQueue = xQueueCreate(100, sizeof(String));
	Serial.println("[MainProcess] - Begin");

	xTaskCreate(
		MainProcess::readDistanceTask,
		"ReadDistance",
		configMINIMAL_STACK_SIZE*4,
		this,
		tskIDLE_PRIORITY+3,
		&readDistanceTaskHandle
	);

	xTaskCreate(
		MainProcess::serialEventTask,
		"SerialEvent",
		configMINIMAL_STACK_SIZE*2,
		this,
		tskIDLE_PRIORITY+2,
		&serialTaskHandle
	);

	xTaskCreate(
		MainProcess::sendQueue,
		"SendQueue",
		configMINIMAL_STACK_SIZE*2,
		this,
		tskIDLE_PRIORITY + 2,
		&sendQueueTaskHandle);

	xTaskCreate(
		MainProcess::handleMessageTask,
		"HandleMessage",
		configMINIMAL_STACK_SIZE*6,
		this,
		tskIDLE_PRIORITY + 1,
		&handleMessageTaskHandle);
	
		
	xTaskCreate(
		MainProcess::processingDeviceTask,
		"MainProcess",
		configMINIMAL_STACK_SIZE*4,
		this	,
		tskIDLE_PRIORITY + 1,
		&processingDeviceTaskHandle
	);

	vTaskStartScheduler();
}

void MainProcess::sendQueue(void* pvParameters) {
	MainProcess* self = static_cast<MainProcess*>(pvParameters);
	Serial.println("[MainProcess] - SendQueue");
	String messageToSend;
	for (;;) {
		if (xQueueReceive(self->sendMessageQueue, &messageToSend, portMAX_DELAY) == pdTRUE) {
            Serial2.println(messageToSend);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

void MainProcess::processingDeviceTask(void* pvParameters) {
    MainProcess* self = static_cast<MainProcess*>(pvParameters);
    
    for (;;) {
		self->processingDevice(self->weight, self->maxSpeed); 
        vTaskDelay(pdMS_TO_TICKS(300));  
    }
}

void MainProcess::handleMessageTask(void* pvParameters) {
	MainProcess* self = static_cast<MainProcess*>(pvParameters);
	for (;;) {
		if(Info.taskActive)
			self->handleMessage(); 
		vTaskDelay(pdMS_TO_TICKS(10));  
	}
}

void MainProcess::serialEventTask(void* pvParameters) {
	MainProcess* self = static_cast<MainProcess*>(pvParameters);
	for (;;) {
		self->serialEvent(); 
		vTaskDelay(pdMS_TO_TICKS(10));  
	}
}

void MainProcess::readDistanceTask(void* pvParameters) {
	MainProcess* self = static_cast<MainProcess*>(pvParameters);
	self->readDistance();
}

void MainProcess::init() {
    // analogReadResolution(12);
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial.println("[MainProcess] - Init");

	const int inputPins[] = {forwardSensor, backwardSensor, updownSensor};
    const int outputPins[] = {motor11Revesal, motor12Revesal, motor2Revesal, motor11PWM, motor12PWM, motor2PWM};

    for (int pin : inputPins) pinMode(pin, INPUT);
    for (int pin : outputPins) {
        pinMode(pin, OUTPUT);
        analogWrite(pin, 0);
    }
}

void MainProcess::handleMessage() {
    if (isCommingMsg.startsWith("CMD:")) {
        handleCommand(isCommingMsg);
    } else if (isCommingMsg.startsWith("DATA:")) {
        handleData(isCommingMsg);
    }
}

void MainProcess::handleCommand(String& command) {
	String direction, speed;
    String cmd = command.substring(4);
	// CMD:forward, CMD:backward, CMD:stop, CMD:lift_box, CMD:drop_box, CMD:clr
    if(cmd == "forward") {
        // Serial.println("[handleCommand] - go_forward");
        if (Info.Motor1_mode != 1) {
            while (true) {
                if (Info.PWM_MT_1 > 10) {
                    Info.PWM_MT_1 -= 5;
                    analogWrite(motor11PWM, Info.PWM_MT_1);
					analogWrite(motor12PWM, Info.PWM_MT_1);
					vTaskDelay(pdMS_TO_TICKS(10));
                } 
				else {
                    Info.PWM_MT_1 = 0;
                    Info.Motor1_mode = 1;
                    break;
                }
            }
        }
    }
    else if (cmd == "backward") {
		// Serial.println("[handelCommand] - go_backward");
		if (Info.Motor1_mode != 2) {
			while (true) {
				if (Info.PWM_MT_1 > 10) {
					Info.PWM_MT_1 -= 5; 
                    analogWrite(motor11PWM, Info.PWM_MT_1);
					analogWrite(motor12PWM, Info.PWM_MT_1);
					vTaskDelay(pdMS_TO_TICKS(10));
				} 
				else { 
					Info.PWM_MT_1 = 0;
					Info.Motor1_mode = 2;
					break;
				}
			}
		}
	} 
	else if (cmd == "stop") {
		// Serial.println("[handelCommand] - stop");
		Info.Motor1_mode = 0;
		Info.Motor2_mode = 0;
    }  
	else if(cmd == "lift_box") {
		// Serial.println("[handleCommand] - up");
		stopMT1();
		if(Info.Motor2_mode == 0) 
			Info.Motor2_mode = 1;

		if(Info.Motor2_mode != 1) {
			while (1){
			    if (Info.PWM_MT_2 > 10) {
                    Info.PWM_MT_2--;
                    analogWrite(motor2PWM, Info.PWM_MT_2);
                    vTaskDelay(pdMS_TO_TICKS(10));
                } 
				else {
                    Info.PWM_MT_2 = 0;
                    Info.Motor2_mode = 1;
                    break;
                }
            }
		}
	} 
	else if(cmd == "drop_box") {
		// Serial.println("[handleCommand] - down");
		stopMT1();
		if (Info.Motor2_mode != 2) {
			while (1){
			    if (Info.PWM_MT_2 > 10) {
                    Info.PWM_MT_2--;
                    analogWrite(motor2PWM, Info.PWM_MT_2);
                    vTaskDelay(pdMS_TO_TICKS(10));
                } 
				else {
                    Info.PWM_MT_2 = 0;
                    Info.Motor2_mode = 2;
                    break;
                }
            }
		}
	} 
	else if (cmd == "clr") {
		// Serial.println("[handleCommand] - clear");
		Info.Motor1_mode = 0;
		Info.Motor2_mode = 0;
		Info.taskActive = false;
		isCommingMsg = "";
        inComingMessage = "";
		return;
	}
    else {
		Serial.println("[handleCommand] - unknown command");
	}
}

void MainProcess::handleData(String& data) {
	String dataStr = data.substring(5);
	// Serial.println("[handleCommand] - handleData");
	// DATA:{"weight":10,"movement_pwm":255}
	JsonDocument doc;
	DeserializationError error = deserializeJson(doc, dataStr);

	// Test if parsing succeeds.
	if (error) {
		Serial.print(F("deserializeJson() failed: "));
		Serial.println(error.f_str());
		return;
	}

	if(doc["max_pwm_movement"] != NULL) {
		maxSpeed = doc["max_pwm_movement"];
	}
	if (doc["parameter_motor"] != NULL) {
		weight = doc["parameter_motor"];
	}
	if (doc["max_distance_move"] != NULL) {
		maxDistanceFw = doc["max_distance_move"];

	}
	if (doc["min_distance_move"] != NULL) {
		minDistanceFw = doc["min_distance_move"];
	}

	// maxSpeed = doc["max_pwm_movement"] 
	// weight = doc["parameter_motor"] | NULL;
	// maxDistanceFw = doc["max_distance_move"] | NULL;
	// minDistanceFw = doc["min_distance_move"] | NULL;

	// Serial.println("[handleCommand] - weight: " + String(weight));
	// Serial.println("[handleCommand] - maxSpeed: " + String(maxSpeed));
}

void MainProcess::processingDevice(int weight, int maxSpeed) {
	// Serial.println("[Device processing] - maxSpeed: " + String(maxSpeed));
	// Serial.println("[Device processing] - weight: " + String(weight));
	//Moving motor processing
	if (Info.Motor1_mode == 1) {
		// Serial.println("[Device processing] - go_forward");
		liftBox();
		digitalWrite(motor11Revesal, LOW);
		digitalWrite(motor12Revesal, LOW);

		motor1Speed = detectTarget(maxSpeed, Info.distanceFW);
		if (motor1Speed < 30){
			Info.PWM_MT_1 = 0;
		}
		else{
			if (motor1Speed > Info.PWM_MT_1)
				Info.PWM_MT_1 += weight;
			else if (motor1Speed < Info.PWM_MT_1)
				Info.PWM_MT_1 -= weight*10;
		}
		//_result_PWM 
		analogWrite(motor11PWM, Info.PWM_MT_1);
		analogWrite(motor12PWM, Info.PWM_MT_1);
		Serial.println("[Device processing] - Motor 1 speed: " + String(Info.PWM_MT_1));
	} 
	else if (Info.Motor1_mode == 2) {
		// Serial.println("[Device processing] - go_backward");
		liftBox();
		digitalWrite(motor11Revesal, HIGH);
		digitalWrite(motor12Revesal, HIGH);

		motor1Speed = detectTarget(maxSpeed, Info.distanceBW);

		if (motor1Speed < 30){
			Info.PWM_MT_1 = 0;
		}
		else{
			if (motor1Speed > Info.PWM_MT_1)
				Info.PWM_MT_1 += weight;
			else if (motor1Speed < Info.PWM_MT_1)
				Info.PWM_MT_1 -= weight*10;
		}
		//_result_PWM 
		analogWrite(motor11PWM, Info.PWM_MT_1);
		analogWrite(motor12PWM, Info.PWM_MT_1);
		Serial.println("[Device processing] - Motor 1 speed  : " + String(Info.PWM_MT_1));
	} 
	else {
		Serial.println("[Device processing] - stop motor 1");
		digitalWrite(motor11Revesal, LOW);
		digitalWrite(motor12Revesal, LOW);
		if(Info.PWM_MT_1 > 0)	Info.PWM_MT_1 -= weight;
		analogWrite(motor11PWM, Info.PWM_MT_1);
		analogWrite(motor12PWM, Info.PWM_MT_1);
	}

	// Up-Down Motor processing
	if (Info.Motor2_mode == 1) {
		Serial.println("[Device processing] - up");
		stopMT1();
		
		digitalWrite(motor2Revesal, HIGH);

		if (Info.distanceUD < 20) {
			motor2Speed = 0;
			if (Info.PWM_MT_2 < 20) {
				Info.PWM_MT_2 = 0;
				Info.Motor2_mode = 0;
			}
			else
				Info.PWM_MT_2 -= (weight * 5);
		} 
		else {
			motor2Speed = (Info.distanceUD / 150) * 255;
			if (motor2Speed > Info.PWM_MT_2) {
				Info.PWM_MT_2 += weight;
			}
		}
		//_result_PWM 
		analogWrite(motor2PWM, Info.PWM_MT_2);
		Serial.println("[Device processing] - Motor 2 speed  : " + String(Info.PWM_MT_2));
	} 
	else if (Info.Motor2_mode == 2) {
		Serial.println("[Device processing] - down");
		stopMT1();

		digitalWrite(motor2Revesal, LOW);
		if (Info.distanceUD > 150){
			motor2Speed = 0;
			Info.Motor2_mode = 0;
			Info.PWM_MT_2 = 0;
		} 
		else {
			motor2Speed = (Info.distanceUD / 150) * 255;

			if (motor2Speed > Info.PWM_MT_2) {
				Info.PWM_MT_2 += weight;
			}
			else 
				Info.PWM_MT_2 -= weight;
		}
		//_result_PWM 
		analogWrite(motor2PWM, Info.PWM_MT_2);
		Serial.println("[Device processing] - Motor 2 speed: " + String(Info.PWM_MT_2));
	} 
	else {
		Serial.println("[Device processing] - stop motor 2");
		digitalWrite(motor2Revesal, LOW);
		if (Info.PWM_MT_2 > 0)	Info.PWM_MT_2 -= weight;
		analogWrite(motor2PWM, Info.PWM_MT_2);
	}
}
void MainProcess::liftBox() {
	if(Info.distanceUD > 20) {
		Serial.println("[Device processing] - Lifting box before moving.");
		Info.Motor2_mode = 1;
	}
}

void MainProcess::readDistance() {
	for (;;) {
		Info.distanceFW = distance(forwardSensor, model1);
		Info.distanceBW = distance(backwardSensor, model1);
		Info.distanceUD = distance(updownSensor, model2);

		JsonDocument doc;
		doc["forward_distance"] = Info.distanceFW;
		doc["backward_distance"] = Info.distanceBW;
		doc["lift_distance"] = Info.distanceUD;

		String buffer;
		serializeJson(doc, buffer);
		String message = "DATA:" + buffer;
		Serial.println(message);

		if (sendMessageQueue != nullptr) {
			xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(10));
		}
    
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MainProcess::handleGetData(){
    Serial.println("[handleCommand] - handleGetData");
    JsonDocument jsonDoc;

	jsonDoc["forward_distance"] = Info.distanceFW;
	jsonDoc["backward_distance"] = Info.distanceBW;
	jsonDoc["lift_distance"] = Info.distanceUD;
	jsonDoc["weight"] = Info.weight;
	jsonDoc["battery"] = Info.battery;
	jsonDoc["movement_motor"] = Info.Motor1_mode;
	jsonDoc["movement_pwm"] = Info.PWM_MT_1;
	jsonDoc["lift_motor"] = Info.Motor2_mode;
	jsonDoc["lift_pwm"] =Info.PWM_MT_2;
	jsonDoc["safety"] = Info.safety;
	jsonDoc["door_state"] = Info.door_state;

	String buffer;
    serializeJson(jsonDoc, buffer);
	String message = "DATA:" + buffer;
	Serial.println(message);
	if (sendMessageQueue != nullptr) {
		xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(10));
	}
}

void MainProcess::stopMT1(){
	if (Info.Motor1_mode != 0) {
		Serial.println("[handleCommand] - Stopping motor 1 before lifting");
		
		while (Info.PWM_MT_1 > 0) {
			Info.PWM_MT_1--;
			analogWrite(motor11PWM, Info.PWM_MT_1);
			analogWrite(motor12PWM, Info.PWM_MT_1);
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		Info.Motor1_mode = 0;
		digitalWrite(motor11Revesal, LOW);
		digitalWrite(motor12Revesal, LOW);
	}
}
void MainProcess::serialEvent(void){
	if (Serial.available()) {
		inComingMessage = Serial.readStringUntil('\n');
		inComingMessage.trim();
		if (inComingMessage.length() > 0) {
			Serial.println("[Serial Event] - Received:" + inComingMessage);
			isCommingMsg = inComingMessage;
			Info.taskActive = true;
			handleMessage();
			inComingMessage = "";
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

float MainProcess::detectTarget(float maxSpeed,float distance){
	if (distance > maxDistanceFw) target = maxSpeed;
	else if (distance < maxDistanceFw && distance > minDistanceFw) target = (distance*maxSpeed)/60;
	else target = 0;
	return target; 
}

 void MainProcess::sort(float a[], int size) {
	for(int i = 0; i < size-1; i++) {
		bool swapped = false;
		for(int j = 0; j < size-i-1; j++) {
			if(a[j] > a[j+1]) {
				float temp = a[j];
				a[j] = a[j+1];
				a[j+1] = temp;
				swapped = true;
			}
		}
		if (!swapped) break;
	}
}

int MainProcess::distance(int8_t sensorName, int16_t modelSensor) {
	float ir_val[SAMPLES] = {};
	int distanceCM;
	int median;

	for (int i=0; i<SAMPLES; i++){
		ir_val[i] = analogRead(sensorName);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	sort(ir_val, SAMPLES);
	median = ir_val[SAMPLES/2];

	if (modelSensor == 1080) 
		if (median > 1000) distanceCM = 0;
		else if (median < 1000 && median > 150)
			distanceCM = 29.988 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.173);
		else 
			distanceCM = 80;
	else if (modelSensor == 20150)
		if (median > 1000 ) distanceCM = 0; 
		else if (median < 1000 && median > 150)
			distanceCM = 60.374 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.16);
		else 
			distanceCM = 150;
	else 
		distanceCM = 0;
	return distanceCM;
}