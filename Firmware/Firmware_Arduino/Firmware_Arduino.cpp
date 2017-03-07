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

int dist1_value = 0;
int dist2_value = 0;
int dist3_value = 0;
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
	int dist1[MAX_STORAGE_SIZE];; // Distance from Sensor 1 (Left Arm)
	int dist2[MAX_STORAGE_SIZE];; // Distance from Sensor 2 (Wand)
	int dist3[MAX_STORAGE_SIZE];; // Distance from Sensor 3 (Right Arm)
	int timestamp[MAX_STORAGE_SIZE];;
} ObstDetectionStorage;

StepCountStorage stepCountStorage;
ObstDetectionStorage obstDetectionStorage;

int sc_pos_packet = 0;
int sc_pos_json = 0;

int od_pos_packet = 0;
int od_pos_json = 0;

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

		Serial.println("getStepCountData");

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

		Serial.println("addStepCountData");

		stepCountStorage.dir[sc_pos_packet] = dir_value;
		stepCountStorage.accelx[sc_pos_packet] = accelx_value;
		stepCountStorage.accely[sc_pos_packet] = accely_value;
		stepCountStorage.accelz[sc_pos_packet] = accelz_value;
		stepCountStorage.timestamp[sc_pos_packet] = sc_timestamp_value;

		sc_pos_packet = (sc_pos_packet + 1) % MAX_STORAGE_SIZE;

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void transmitStepCountData(void *p) {
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		int sc_pos_backup = sc_pos_json;

		Serial.println("transmitStepCountData");

		StaticJsonBuffer<512> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& dir = json.createNestedArray("dir");
		JsonArray& accelx = json.createNestedArray("accelx");
		JsonArray& accely = json.createNestedArray("accely");
		JsonArray& accelz = json.createNestedArray("accelz");
		JsonArray& timestamp = json.createNestedArray("timestamp");
		int checksum = 0;

		for (i = 0; (sc_pos_json != sc_pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			dir.add(stepCountStorage.dir[sc_pos_json]);
			accelx.add(stepCountStorage.accelx[sc_pos_json]);
			accely.add(stepCountStorage.accely[sc_pos_json]);
			accelz.add(stepCountStorage.accelz[sc_pos_json]);
			timestamp.add(stepCountStorage.timestamp[sc_pos_json]);
			checksum = (checksum + stepCountStorage.timestamp[sc_pos_json]) %256;
			sc_pos_json = (sc_pos_json + 1) % MAX_STORAGE_SIZE;
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
				sc_pos_json = sc_pos_backup;
			}
			Serial.println(msg);
		}
		else {
			Serial.println("error");
			sc_pos_json = sc_pos_backup;
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelay(100);
	}
}

void getObstDetectionData(void *p){
	// TO BE COMPLETED BY HARDWARE
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		Serial.println("getObstDetectionData");

		dist1_value = dist1_value + 10; // dist1_value = analogRead(PIN_DIST1);
		dist2_value = dist2_value + 10; // dist2_value = analogRead(PIN_DIST2);
		dist3_value = dist3_value + 10; // dist3_value = analogRead(PIN_DIST3);
		od_timestamp_value = od_timestamp_value; // od_timestamp_value = getTime();

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}


void addObstDetectionData(void *p) {
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		Serial.println("addObstDetectionData");

		obstDetectionStorage.dist1[od_pos_packet] = dist1_value;
		obstDetectionStorage.dist2[od_pos_packet] = dist2_value;
		obstDetectionStorage.dist3[od_pos_packet] = dist3_value;
		obstDetectionStorage.timestamp[od_pos_packet] = od_timestamp_value;

		od_pos_packet = (od_pos_packet + 1) % MAX_STORAGE_SIZE;

		xSemaphoreGive(xSemaphore);
		vTaskDelay(20);
	}
}

void transmitObstDetectionData(void *p) {

	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		int od_pos_backup = od_pos_json;

		Serial.println("transmitObstDetectionData");

		StaticJsonBuffer<512> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& dist1 = json.createNestedArray("dist1");
		JsonArray& dist2 = json.createNestedArray("dist2");
		JsonArray& dist3 = json.createNestedArray("dist3");
		JsonArray& timestamp = json.createNestedArray("timestamp");
		int checksum = 0;

		for (i = 0; (od_pos_json != od_pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			dist1.add(obstDetectionStorage.dist1[od_pos_json]);
			dist2.add(obstDetectionStorage.dist2[od_pos_json]);
			dist3.add(obstDetectionStorage.dist3[od_pos_json]);
			timestamp.add(obstDetectionStorage.timestamp[od_pos_json]);
			checksum = (checksum + obstDetectionStorage.timestamp[od_pos_json]) %256;
			od_pos_json = (od_pos_json + 1) % MAX_STORAGE_SIZE;
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
				od_pos_json = od_pos_backup;
			}
			Serial.println(msg);
		}
		else {
			Serial.println("error");
			od_pos_json = od_pos_backup;
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelay(100);
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

//	xTaskCreate(getObstDetectionData, "getObstDetectionData", STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(addObstDetectionData, "addObstDetectionData", STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate(transmitObstDetectionData, "transmitObstDetectionData", STACK_SIZE, NULL, 3, NULL);

	xTaskCreate(getStepCountData, "getStepCountData", STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(addStepCountData, "addStepCountData", STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(transmitStepCountData, "transmitStepCountData", STACK_SIZE, NULL, 6, NULL);
}

void loop() {

}
