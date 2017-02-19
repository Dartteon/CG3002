#include <ArduinoJson.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#include <Arduino.h>
#include <avr/io.h>
#include <task.h>
#include <semphr.h>

#define PIN_PTTM 0
#define STACK_SIZE 1024
#define MAX_STACK_SIZE 20

typedef struct packetStrt {
	//int type;
	int id[MAX_STACK_SIZE];
	int data[MAX_STACK_SIZE];
	int checksum = 0;
} Packet;

Packet newPacket;
StaticJsonBuffer<200> jsonBuffer;
JsonObject& json = jsonBuffer.createObject();
JsonArray& id = json.createNestedArray("id");
JsonArray& data = json.createNestedArray("data");

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
					Serial.println("msg");
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
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		//Serial.println("addData");

		newPacket.id[pos_packet] = id1++;
		newPacket.data[pos_packet] = data1++;
		newPacket.checksum++;

		//Serial.println(newPacket.id[counter]);

		pos_packet = (pos_packet + 1) % MAX_STACK_SIZE;
		xSemaphoreGive(xSemaphore);

		vTaskDelay(20);
	}

}

void transmit(void *p) {
//	Packet packet;
	int i = 0;
	for (;;) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		Serial.println("sendData");

		StaticJsonBuffer<256> jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		JsonArray& id = json.createNestedArray("id");
		JsonArray& data = json.createNestedArray("data");

		for (i = 0; (pos_json != pos_packet) && (i < 10); i++) {
			id.add(newPacket.id[pos_json]);
			data.add(newPacket.data[pos_json]);
			pos_json = (pos_json + 1) % MAX_STACK_SIZE;
		}
		json["checksum"] = newPacket.checksum++;
		json.printTo(Serial1);
		//Serial1.println("");
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

