#include <ArduinoJson.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#include <Arduino.h>
#include <avr/io.h>
#include <task.h>
#include <semphr.h>

//Hardware
#define PIN_TEST_SWITCH 10

//Firmware
#define STACK_SIZE 1024
#define MAX_STORAGE_SIZE 100
#define MAX_SENDING_PACKET_SIZE 10

// Hardware Variables
int dir_value = 0;
int accelx_value = 0;
int accely_value = 0;
int accelz_value = 0;
int sc_timestamp_value = 0;

int dist1_value = 100;
int dist2_value = 100;
int dist3_value = 100;
int od_timestamp_value = 0;


// Firmware Variables
typedef struct stepCountStorageStrt {
	int dir[MAX_STORAGE_SIZE];
	int accelx[MAX_STORAGE_SIZE];
	int accely[MAX_STORAGE_SIZE];
	int accelz[MAX_STORAGE_SIZE];
	int timestamp[MAX_STORAGE_SIZE];
} StepCountStorage;

typedef struct obstDetectionStrt {
	int dist1; // Distance from Sensor 1 (Left Arm)
	int dist2; // Distance from Sensor 2 (Wand)
	int dist3; // Distance from Sensor 3 (Right Arm)
	int timestamp;
} ObstDetectionStorage;

StepCountStorage stepCountStorage;
ObstDetectionStorage obstDetectionStorage;

int pos_packet = 0;
int pos_json = 0;

static BaseType_t xHigherPriorityTaskWoken;

SemaphoreHandle_t xSemaphore = NULL;

QueueHandle_t xQueue = NULL;

/*
 void vprint(void *p){
	 xSemaphoreTake(xSemaphorePrint, portMAX_DELAY);
	 Serial.println(*((String*) p));
	 xSemaphoreGive(xSemaphorePrint);
 }
 */

static void initialize() {
	bool initFlag = true;

	while (initFlag) {
		if (Serial1.available()) {
			int msg = Serial1.read();
			Serial.println((char) msg);
			//Serial.println("1");
			if (msg == 'h') {
				msg = 'a';
				Serial1.write(msg);

				Serial.println((char) msg);
				if (msg == 'a') {
					initFlag = false;
					//Serial.println("2");
				} else{
					Serial.print("error 3: ");
					Serial.println(msg);
				}
			}
		}
	}
}

void getStepCountData(void *p){
	// TO BE COMPLETED BY HARDWARE
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

//		dir_value = digitalRead(PIN_TEST_SWITCH); // tested and worked successfully
		dir_value = dir_value + 1; // dir_value = analogRead(PIN_DIR);
		accelx_value = accelx_value + 1; // accelx_value = analogRead(PIN_ACCELX);
		accely_value = accely_value + 1; // accely_value = analogRead(PIN_ACCELY);
		accelz_value = accelz_value + 1; // accelz_value = analogRead(PIN_ACCELZ);
		sc_timestamp_value = sc_timestamp_value + 1; // sc_timestamp_value = getTime();

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void addStepCountData(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		//Serial.println("addStepCountData");
		stepCountStorage.dir[pos_packet] = dir_value;
		stepCountStorage.accelx[pos_packet] = accelx_value;
		stepCountStorage.accely[pos_packet] = accely_value;
		stepCountStorage.accelz[pos_packet] = accelz_value;
		stepCountStorage.timestamp[pos_packet] = sc_timestamp_value;

		pos_packet = (pos_packet + 1) % MAX_STORAGE_SIZE;

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void transmitStepCountData(void *p) {
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		int pos_backup = pos_json;
		Serial.println("sendStepCountData");

		StaticJsonBuffer<512> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& dir = json.createNestedArray("dir");
		JsonArray& accelx = json.createNestedArray("accelx");
		JsonArray& accely = json.createNestedArray("accely");
		JsonArray& accelz = json.createNestedArray("accelz");
		JsonArray& timestamp = json.createNestedArray("timestamp");
		int checksum = 0;

		for (i = 0; (pos_json != pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			dir.add(stepCountStorage.dir[pos_json]);
			accelx.add(stepCountStorage.accelx[pos_json]);
			accely.add(stepCountStorage.accely[pos_json]);
			accelz.add(stepCountStorage.accelz[pos_json]);
			timestamp.add(stepCountStorage.timestamp[pos_json]);
			checksum = (checksum + stepCountStorage.dir[pos_json]) %256;
			pos_json = (pos_json + 1) % MAX_STORAGE_SIZE;
		}

		json["checksum"] = checksum;

		//might fuck up
		while(!Serial1){

		}

		json.printTo(Serial1);

		//might fuck up
		while (!(Serial1 && Serial1.available())){

		}

		if (Serial1.available()){
			char msg = Serial1.read();
			if (msg =='n'){
				pos_json = pos_backup;
			}
			Serial.println(msg);
		}
		else {
			Serial.println("error");
			pos_json = pos_backup;
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelay(100);
	}
}

void getObstDetectionData(void *p){
	// TO BE COMPLETED BY HARDWARE
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

//		dist1_value = digitalRead(PIN_TEST_SWITCH); // yet to test
		dist1_value = dist1_value + 1; // dist1_value = analogRead(PIN_DIST1);
		dist2_value = dist2_value; // dist2_value = analogRead(PIN_DIST2);
		dist3_value = dist3_value; // dist3_value = analogRead(PIN_DIST3);
		od_timestamp_value = od_timestamp_value; // od_timestamp_value = getTime();

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}


void addObstDetectionData(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		//Serial.println("addObstDetectionData");

		obstDetectionStorage.dist1 = dist1_value;
		obstDetectionStorage.dist2 = dist2_value;
		obstDetectionStorage.dist3 = dist3_value;
		obstDetectionStorage.timestamp = od_timestamp_value;

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void transmitObstDetectionData() {
	// %5 is just for example. when integrating, hardware should tell firmware the condition to interrupt
	if ((obstDetectionStorage.dist1 % 5) == 0) {
		//interrupt
		for (;;) {
			xSemaphoreTake(xSemaphore, portMAX_DELAY);
			Serial.println("sendObstDetectionData");

			StaticJsonBuffer<256> jsonBuffer;
			JsonObject& json = jsonBuffer.createObject();

			int checksum = 0;
			checksum = (checksum + obstDetectionStorage.dist1) %256;

			json["dist1"] = obstDetectionStorage.dist1;
			json["dist2"] = obstDetectionStorage.dist2;
			json["dist3"] = obstDetectionStorage.dist3;
			json["timestamp"] = obstDetectionStorage.timestamp;
			json["checksum"] = checksum;

			//might fuck up
			while(!Serial1){

			}

			json.printTo(Serial1);

			//might fuck up
			while (!(Serial1 && Serial1.available())){

			}

			if (Serial1.available()){
				char msg = Serial1.read();
				if (msg =='n'){
					//something bad happened
				}
				Serial.println(msg);
			}
			else {
				Serial.println("error");
				//something bad happened
			}

			xSemaphoreGive(xSemaphore);
			vTaskDelay(100);
		}
	}
}

void setup() {
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);


	Serial.begin(9600);
	Serial1.begin(9600);

	Serial.flush();
	Serial1.flush();

	Serial.println("begin");
	initialize();
	Serial.println("begin2");

//	pinMode(PIN_TEST_SWITCH, INPUT);

	xTaskCreate(getObstDetectionData, "getObstDetectionData", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(addObstDetectionData, "addObstDetectionData", STACK_SIZE, NULL, 2, NULL);

	xTaskCreate(getStepCountData, "getStepCountData", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(addStepCountData, "addStepCountData", STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(transmitStepCountData, "sendStepCountData", STACK_SIZE, NULL, 3, NULL);
}

void loop() {

}
