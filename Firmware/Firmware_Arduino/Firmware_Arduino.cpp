#include <ArduinoJson.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#include <Arduino.h>
#include <avr/io.h>
#include <task.h>
#include <semphr.h>

#define PIN_PTTM 0
#define STACK_SIZE 1024
#define MAX_STORAGE_SIZE 100
#define MAX_SENDING_PACKET_SIZE 10

typedef struct storageStrt {
	int dir[MAX_STORAGE_SIZE];
	int accelx[MAX_STORAGE_SIZE];
	int accely[MAX_STORAGE_SIZE];
	int accelz[MAX_STORAGE_SIZE];
} Storage;

typedef struct distStorageStrt {
	int dist1[MAX_STORAGE_SIZE]; //Distance from Sensor 1 (Left Arm)
	int dist2[MAX_STORAGE_SIZE]; //Distance from Sensor 2 (Wand)
	int dist3[MAX_STORAGE_SIZE]; //Distance from Sensor 3 (Right Arm)
} DistStorage;


Storage mainStorage; //Will probably rename another time to dirAccelStorage
DistStorage distStorage;

int pos_packet = 0;
int pos_json = 0;

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

void addData(void *p) {
	int dir1 = 1;
	int accelx1 = 0;
	int accely1 = 0;
	int accelz1 = 0;

	for (;;) {

		//Serial.println("addData");

		mainStorage.dir[pos_packet] = dir1++;
		mainStorage.accelx[pos_packet] = accelx1;
		mainStorage.accely[pos_packet] = accely1;
		mainStorage.accelz[pos_packet] = accelz1;

		pos_packet = (pos_packet + 1) % MAX_STORAGE_SIZE;

		vTaskDelay(20);
	}

}

void addDistanceData(void *p) {
	//dataUnit temp = *((dataUnit*) p);
	int dist1 = 1;
	int dist2 = 1;
	int dist3 = 0;

	for (;;) {
		distStorage.dist1[pos_packet] = dist1++;
		distStorage.dist2[pos_packet] = dist2;
		distStorage.dist3[pos_packet] = dist3;

		//Serial.println(newPacket.id[counter]);

		pos_packet = (pos_packet + 1) % MAX_STORAGE_SIZE;

		vTaskDelay(20);
	}

}


void transmit(void *p) {
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		int pos_backup = pos_json;
		Serial.println("sendData");

		StaticJsonBuffer<256> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& dir = json.createNestedArray("dir");
		JsonArray& accelx = json.createNestedArray("accelx");
		JsonArray& accely = json.createNestedArray("accely");
		JsonArray& accelz = json.createNestedArray("accelz");
		int checksum = 0;

		for (i = 0; (pos_json != pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			dir.add(mainStorage.dir[pos_json]);
			accelx.add(mainStorage.accelx[pos_json]);
			accely.add(mainStorage.accely[pos_json]);
			accelz.add(mainStorage.accelz[pos_json]);
			checksum = (checksum + mainStorage.dir[pos_json]) %256;
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
//test

	xTaskCreate(addData, "addData", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(transmit, "sendData", STACK_SIZE, NULL, 2, NULL);
}

void loop() {

}
