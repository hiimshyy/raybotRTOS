#include "mainProcess.h"

Robot_info Info;
String inComingMessage;
float forwardDistance = 0, backwardDistance = 0, downDistance = 0, upDistance = 0, updownDistance =0;
float motor1Speed, motor2Speed;
float totalFW = 0, totalBW = 0, totalUD = 0, readDistanceCounter = 0;

void MainProcess::begin() {
    init();
	sendMessageQueue = xQueueCreate(100, sizeof(String));
	Serial.println("[MainProcess] - Begin");

	xTaskCreate(
		MainProcess::sendQueue,
		"SendQueue",
		configMINIMAL_STACK_SIZE*2,
		this,
		tskIDLE_PRIORITY + 1,
		&sendQueueTaskHandle);

	xTaskCreate(
		MainProcess::handleMessageTask,
		"HandleMessage",
		configMINIMAL_STACK_SIZE*10,
		this,
		tskIDLE_PRIORITY + 2,
		&handleMessageTaskHandle);
	xTaskCreate(
		MainProcess::readDistanceTask,
		"ReadDistance",
		configMINIMAL_STACK_SIZE*2,
		this,
		tskIDLE_PRIORITY+1,
		&readDistanceTaskHandle
	);
		
	xTaskCreate(
		MainProcess::processingDeviceTask,
		"MainProcess",
		configMINIMAL_STACK_SIZE*8,
		this	,
		tskIDLE_PRIORITY + 2,
		&processingDeviceTaskHandle
	);

	xTaskCreate(
		MainProcess::serialEventTask,
		"SerialEvent",
		configMINIMAL_STACK_SIZE*2,
		this,
		tskIDLE_PRIORITY,
		&serialTaskHandle
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
        self->processingDevice(5); 
        vTaskDelay(pdMS_TO_TICKS(100));  
    }
}

void MainProcess::handleMessageTask(void* pvParameters) {
	MainProcess* self = static_cast<MainProcess*>(pvParameters);
	for (;;) {
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
    analogReadResolution(12);
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial.println("[MainProcess] - Init");

	const int inputPins[] = {};
    const int outputPins[] = {_revesal_motor_1, _revesal_motor_2, _pwm_motor_1, _pwm_motor_2};

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
    String cmd = command.substring(4);
	Serial.println("[handleCommand] - " + cmd);
    if(cmd == "forward") {
        Serial.println("[handelCommand] - go_forward");
		liftBox();
        // Set motor to go forward
        if (Info.Motor1_mode == 0) {
            Info.Motor1_mode = 1;
        }
        // If the motor is running in reverse, gradually reduce speed, then go forward
        else if (Info.Motor1_mode != 1) {
            while (true) {
                if (Info.PWM_MT_1 > 10) {
                    Info.PWM_MT_1 -= 5;
                    analogWrite(_pwm_motor_1, Info.PWM_MT_1);
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
		Serial.println("[handelCommand] - go_backward");
		liftBox();
        if (Info.Motor1_mode == 0){
			Info.Motor1_mode = 2;
		}
		// neu motor dang chay lui thi giam toc sau do se chay thang
		else if (Info.Motor1_mode != 2) {
			// giảm tốc độ động cơ
			while (true) {
				if (Info.PWM_MT_1 > 10) {
					Info.PWM_MT_1 -= 10; 
                    analogWrite(_pwm_motor_1, Info.PWM_MT_1);
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
		Serial.println("[handelCommand] - stop");
        if (Info.Motor1_mode != 0){
			// giảm tốc độ động cơ
			while (true){
				if (Info.PWM_MT_1 > 10){
					Info.PWM_MT_1 -= 5;
					analogWrite(_pwm_motor_1, Info.PWM_MT_1);
					vTaskDelay(pdMS_TO_TICKS(10));;
				} else{
					Info.PWM_MT_1 = 0;
					Info.Motor1_mode = 0;
					break;
				}
			}
		}
		motor1Stage = 0;

		if (Info.Motor2_mode != 0){
			// giảm tốc độ động cơ
			while (true){
				if (Info.PWM_MT_2 > 10){
					Info.PWM_MT_2 -= 5;
					analogWrite(_pwm_motor_2, Info.PWM_MT_2);
					vTaskDelay(pdMS_TO_TICKS(10));;
				} else{
					Info.PWM_MT_2 = 0;
					Info.Motor2_mode = 0;
					break;
				}
			}
		}
		motor2Stage = 0;
    } 
	else if(cmd == "clr") {
		Serial.println("[handelCommand] - clr");
        if (Info.Motor1_mode != 0 && Info.PWM_MT_1 != 0) {
			// giảm tốc độ động cơ
			while (true) {
				if (Info.PWM_MT_1 > 10) {
					Info.PWM_MT_1--; 
					analogWrite(_pwm_motor_1, Info.PWM_MT_1);
					vTaskDelay(pdMS_TO_TICKS(10));;
				} 
				else {
					Info.PWM_MT_1 = 0;
					Info.Motor1_mode = 0;
					break;
				}
			}
		} else {
			Info.PWM_MT_1 = 0;
			Info.Motor1_mode = 0;
		}
		motor1Stage = 0;
    } 
	else if(cmd == "lift_box") {
		Serial.println("[handleCommand] - up");
		stopMT1();
		if(Info.Motor2_mode == 0) 
			Info.Motor2_mode = 1;

		// neu dang tha hang, giam toc do va nang len
		if(Info.Motor2_mode == 2) {
			while (1){
			    if (Info.PWM_MT_2 > 10) {
                    Info.PWM_MT_2--;
                    analogWrite(_pwm_motor_2, Info.PWM_MT_2);
                    vTaskDelay(pdMS_TO_TICKS(10));;
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
		Serial.println("[handleCommand] - down");
		stopMT1();
		if (Info.Motor2_mode == 0)
			Info.Motor2_mode = 2;
		if (Info.Motor2_mode == 1) {
			while (1){
			    if (Info.PWM_MT_2 > 10) {
                    Info.PWM_MT_2--;
                    analogWrite(_pwm_motor_2, Info.PWM_MT_2);
                    vTaskDelay(pdMS_TO_TICKS(10));;
                } 
				else {
                    Info.PWM_MT_2 = 0;
                    Info.Motor2_mode = 2;
                    break;
                }
            }
		}
	} 
    else {
		Serial.println("[handleCommand] - unknown command");
	}
}

void MainProcess::handleData(String& data) {


}

void MainProcess::processingDevice(int weights) {
	// Serial.println("[Device processing] - Forward distance: " + String(Info.distanceFW));
	// Serial.println("[Device processing] - Backward distance: " + String(Info.distanceBW));
	// Serial.println("[Device processing] - UpDown  distance: " + String(Info.distanceUD));

	if (Info.distanceFW < 1200)	
		forwardDistance = 89;	
	else 	  
		forwardDistance = 29; 

	if (Info.distanceBW < 1200)	
		backwardDistance = 89;
	else
		backwardDistance = 29;

	//Moving motor processing
	if (Info.Motor1_mode == 1) {
		Serial.println("[Device processing] - go_forward");
		if(updownDistance < 2050)
			Info.Motor2_mode = 1;

		digitalWrite(_revesal_motor_1, LOW);

		if (forwardDistance < 30) { 
			motor1Speed = 0;
			if (Info.PWM_MT_1 < 10)
				Info.PWM_MT_1 = 0;
			else
				Info.PWM_MT_1 -= weights;
		} 
		else {
			motor1Speed = (forwardDistance / 150) * 255;

			if (motor1Speed > Info.PWM_MT_1) {
				if (Info.PWM_MT_1 < 250)
					Info.PWM_MT_1 += weights;
				else
					Info.PWM_MT_1 = 255;
			} else {
				if (Info.PWM_MT_1 < 10)
					Info.PWM_MT_1 = 0;
				else
					Info.PWM_MT_1 -= weights;
			}
		}
		//_result_PWM 
		analogWrite(_pwm_motor_1, Info.PWM_MT_1);
		Serial.println("[Device processing] - Motor 1 speed: " + String(Info.PWM_MT_1));
	} 
	else if (Info.Motor1_mode == 2) {
		Serial.println("[Device processing] - go_backward");
		if(updownDistance < 2050)
			Info.Motor2_mode = 1;

		digitalWrite(_revesal_motor_1, HIGH);

		if (backwardDistance < 30) {
			motor1Speed = 0;
			if (Info.PWM_MT_1 < 10)
				Info.PWM_MT_1 = 0;
			else
				Info.PWM_MT_1 -= weights;
		} 
		else {
			motor1Speed = (backwardDistance / 150) * 255;
			if (motor1Speed > Info.PWM_MT_1) {
				if (Info.PWM_MT_1 < 250)
					Info.PWM_MT_1 += weights;
				else
					Info.PWM_MT_1 = 255;
			} 
			else {
				if (Info.PWM_MT_1 < 10)
					Info.PWM_MT_1 = 0;
				else
					Info.PWM_MT_1 -= weights;
			}
		}
		//_result_PWM 
		analogWrite(_pwm_motor_1, Info.PWM_MT_1);
		Serial.println("[Device processing] - Motor 1 speed  : " + String(Info.PWM_MT_1));
	} 
	else {
		Serial.println("[Device processing] - stop motor 1");
		digitalWrite(_revesal_motor_1, LOW);
		Info.PWM_MT_1 = 0;
		analogWrite(_pwm_motor_1, Info.PWM_MT_1);
	}

	// Up-Down Motor processing
	if (Info.Motor2_mode == 1) {
		Serial.println("[Device processing] - up");
		upDistance = Info.distanceUD;

		if (upDistance > 2050)
			upDistance = 19;
		else 
			upDistance = 150;
		digitalWrite(_revesal_motor_2, HIGH);

		if (upDistance < 20) {
			motor2Speed = 0;
			if (Info.PWM_MT_2 < 20) {
				Info.PWM_MT_2 = 0;
				Info.Motor2_mode = 0;
			}
			else
				Info.PWM_MT_2 -= (weights * 5);
		} 
		else {
			motor2Speed = (upDistance / 150) * 255;
			if (motor2Speed > Info.PWM_MT_2) {
				if (Info.PWM_MT_2 < 250)
					Info.PWM_MT_2 += weights;
				else
					Info.PWM_MT_2 = 255;
			} 
			else {
				if (Info.PWM_MT_2 < 50)
					Info.PWM_MT_2 = 0;
				else
					Info.PWM_MT_2 -= (weights * 5);
			}
		}
		//_result_PWM 
		analogWrite(_pwm_motor_2, Info.PWM_MT_2);
		Serial.println("[Device processing] - Motor 2 speed  : " + String(Info.PWM_MT_2));
		
	} 
	else if (Info.Motor2_mode == 2) {
		Serial.println("[Device processing] - down");
		downDistance = Info.distanceUD;

		if (downDistance > 100)
			downDistance = 150;
		else 
			downDistance = 19;

		digitalWrite(_revesal_motor_2, LOW);
		if (downDistance  < 20){
			motor2Speed = 0;
			Info.Motor2_mode = 0;
			Info.PWM_MT_2 = 0;
		} 
		else {
			motor2Speed = (downDistance / 150) * 255;

			if (motor2Speed > Info.PWM_MT_2) {//neu tốc độ cực đại > tăng tốc độ hiện tại thi tăng tốc thêm bằng trọng số
				if (Info.PWM_MT_2 < 250)
					Info.PWM_MT_2 += weights;
				else
					Info.PWM_MT_2 = 255;
			} 
			else {
				if (Info.PWM_MT_2 < 50)
					Info.PWM_MT_2 = 0;
				else
					Info.PWM_MT_2 -= (weights * 5);
			}
		}
		//_result_PWM 
		analogWrite(_pwm_motor_2, Info.PWM_MT_2);
		Serial.println("[Device processing] - Motor 2 speed: " + String(Info.PWM_MT_2));
	} 
	else {
		Serial.println("[Device processing] - stop motor 2");
		digitalWrite(_revesal_motor_2, LOW);
		Info.PWM_MT_2 = 0;
		analogWrite(_pwm_motor_2, Info.PWM_MT_2);
	}
}

void MainProcess::readDistance() {
    const int samples = 10;
	for (;;) {
        // Read analog values
        readDistanceCounter++;
        totalFW += analogRead(_forward_distance_sensor);
        totalBW += analogRead(_backward_distance_sensor);
        totalUD += analogRead(_up_down_distance_sensor);

        if (readDistanceCounter == samples) {
            Info.distanceFW = totalFW / samples;
            Info.distanceBW = totalBW / samples;
            Info.distanceUD = totalUD / samples;

            // Prepare JSON
            JsonDocument doc;
            doc["forward_distance"] = Info.distanceFW;
            doc["backward_distance"] = Info.distanceBW;
            doc["lift_distance"] = Info.distanceUD;

            // Serialize to string
            String buffer;
            serializeJson(doc, buffer);
			String message = "DATA:" + buffer;
			Serial.println(message);

            // Send to queue with timeout
            if (sendMessageQueue != nullptr) {
                xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(100));
            }

            // Reset counters
            totalFW = 0;
            totalBW = 0;
            totalUD = 0;
            readDistanceCounter = 0;
        }

        // Add a small delay to prevent tight looping
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
		xQueueSend(sendMessageQueue, &message, pdMS_TO_TICKS(100));
	}
}
void MainProcess::liftBox(){
    if (updownDistance < 2050) {
        Serial.println("[handleCommand] - Lifting box before moving.");
        Info.Motor2_mode = 1;
    }
}

void MainProcess::stopMT1(){
	if (Info.Motor1_mode != 0) {
		Serial.println("[handleCommand] - Stopping forward/backward motor before lifting");
		
		while (Info.PWM_MT_1 > 0) {
			Info.PWM_MT_1 -= 8;
			analogWrite(_pwm_motor_1, Info.PWM_MT_1);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		Info.Motor1_mode = 0;
		digitalWrite(_revesal_motor_1, LOW);
	}
}
void MainProcess::serialEvent(void){
	if (Serial2.available()) {
		inComingMessage = Serial2.readStringUntil('\n');
		inComingMessage.trim();
		if (inComingMessage.length() > 0) {
			Serial.println("[Serial Event] - Received:" + inComingMessage);
			isCommingMsg = inComingMessage;
			handleMessage();
			inComingMessage = "";
			isCommingMsg = "";
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}