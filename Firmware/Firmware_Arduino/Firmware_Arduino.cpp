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
	//int type;
	int id[MAX_STORAGE_SIZE];
	int data[MAX_STORAGE_SIZE];
} Storage;

Storage mainStorage;

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
	//dataUnit temp = *((dataUnit*) p);
	int id1 = 0;
	int data1 = 1;

	for (;;) {

		//Serial.println("addData");

		mainStorage.id[pos_packet] = id1++;
		mainStorage.data[pos_packet] = data1++;

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
		JsonArray& id = json.createNestedArray("id");
		JsonArray& data = json.createNestedArray("data");
		int checksum = 0;

		for (i = 0; (pos_json != pos_packet) && (i < MAX_SENDING_PACKET_SIZE); i++) {
			id.add(mainStorage.id[pos_json]);
			data.add(mainStorage.data[pos_json]);
			checksum = (checksum + mainStorage.data[pos_json]) %256;
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

/*
 void produce2(void *p) {
 int counter = 1;
 for (;;) {
 counter += 2;
 if (xQueueSendToBack(xQueue, (void * ) &counter,
 (TickType_t ) 0) != pdPASS) {
 xSemaphoreTake(xSemaphore, portMAX_DELAY);
 Serial.println("Message Queue Full");
 xSemaphoreGive(xSemaphore);
 }
 }
 }
 */

/*
 void transmit(void *p) {
 int counter = 1;
 Packet packetSend;
 for (;;) {
 if (xQueueReceive(xQueue, &packetSend,
 (TickType_t) portMAX_DELAY) == pdTRUE) {
 xSemaphoreTake(xSemaphore, portMAX_DELAY);
 Serial.println("begin sendData");
 Serial.println("Packet number: " + counter);
 for (int i = 0; i < MAX_STACK_SIZE; i++) {
 Serial.println(packetSend.id[i]);
 Serial.println(packetSend.data[i]);
 }
 Serial.println(packetSend.checksum);
 xSemaphoreGive(xSemaphore);
 }
 }
 }
 */
