# ISTRUMENTACAO-E-SIMULACAO-DE-SENSORES-PARA-VISUALIZACAO-NUM-DISPLAY
/*Nome ALUNO - João Adam
Nome ALUNO - João Figueiredo
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEEC- Licenciatura em Engenharia Eletrotécnica e de Computadores
SEEV - Engenharia Automovel

TP1: Pretende-se  neste  trabalho  prático  a implementação  e simulação de sensores para a visualização num display, utilizando um sistema operativo de tempo real FreeRTOS. 

LINK: https://www.youtube.com/watch?v=RaGeDIsRlT4&ab_channel=Jo%C3%A3oPedroVasconcelosFigueiredo

*/

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

/*#include "Arduino.h"
 #include "stdio.h"
 #include "freertos/FreeRTOS.h"
 #include "Tasks.h"
 #include "math.h"*/
//#include "easynextionlibrary.h"
#define portENTER_CRITICAL()
#define portEXIT_CRITICAL()

/*Constantes do motor*/
#define maxWaterTemp 130
#define minWaterTemp -10
#define firstWaterTempWarning 105
#define maxrpm 7000
#define minrpm 0
#define num_cil 4
#define tensaomax 150
#define tensaomin 0
#define tpsmax 150
#define tpsmin 0
#define combmax 60
#define combmin 0
/* Definicao do conatantes de tarefas */

#define WaterTempReadPeriod 500
#define voltagePeriod 500
#define FUELPERIOD 500
#define rpmPeriod 50
#define chronoPeriod 500
#define tpsPeriod 500
#define DEBOUNCE_TIME 100

/*Definicao  de ADC*/
#define ADCres 12

/*Definicao de pinos */
#define ignicao 12
#define navigationButton 26
#define chronoButtonStart 25
#define chronoButtonReset 27
//#define rpmSensor 32
#define waterTemperatureSensor 35
#define oilpressure 14
#define tensaoBateria 13
//#define tps 3
#define NivelComb 32

const uint8_t interruptPin = 2;

/*Definicoes de menu */

//#define maxOfOptions 4
//Prototipo dee tarefas
void vMenuShift(void *pvParameters);
void vChronoStartStop(void *pvParameters);
void vChronoReset(void *pvParameters);
void vWaterTemperature(void *pvParameters);
void vTimeKeeping(void *pvParameters);
void vDisplay1(void *pvParameters);
//void vRpm(void *pvParameters);
void vOilpressure(void *pvParameters);
void vIgnition(void *pvParameters);
void vTensaoBateria(void *pvParameters);
void vCombustivel(void *pvParameters);
static void vHandlerTask(void *pvParameters);
//void vTPS(void *pvParameters);

//Variaveis globais para o cronometro
unsigned long start_time = 0;  // Tempo inicial do cronómetro
unsigned long current_time = 0;  // Tempo atual do cronómetro
bool running = false;  // Indica se o cronómetro está a correr
bool started = false;  // Indica se o cronómetro já foi iniciado
unsigned long pause_time = 0; // Tempo quando o cronómetro foi pausado

/*Prototipos de interrupcoes*/
void IRAM_ATTR menuShift();
void IRAM_ATTR chronoStartStop();
void IRAM_ATTR chronoReset();
static void IRAM_ATTR vExampleInterruptHandler();

/* Protoripoa de semaforos para sincronizacao de interrupcoees*/

SemaphoreHandle_t menuSemaphore;
SemaphoreHandle_t chrono_Start_StopSemaphore;
SemaphoreHandle_t chrono_ResetSemaphore;
SemaphoreHandle_t xCountingSemaphore;
SemaphoreHandle_t mutex;

//Prototipo de criacao de Queues


QueueHandle_t rpmQueue;
QueueHandle_t waterTempQueue;
QueueHandle_t chronoToDisplayQueue;
QueueHandle_t oilpressureQueue;
QueueHandle_t ignitionQueue;
QueueHandle_t BatvoltageQueue;
QueueHandle_t CombustivelQueue;

typedef struct {

	unsigned short int hours;
	unsigned short int minutes;
	unsigned short int seconds;
	unsigned short int tenthOfASeconds;

} chronoToDisplay;

//HardwareSerial SerialPort(2); // use UART2

void setup() {
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	//Definicao de semaforos
	menuSemaphore = xSemaphoreCreateBinary();
	chrono_Start_StopSemaphore = xSemaphoreCreateBinary();
	chrono_ResetSemaphore = xSemaphoreCreateBinary();
	xCountingSemaphore = xSemaphoreCreateCounting(100, 0);
	mutex = xSemaphoreCreateMutex();

	/*ADC*/
	analogReadResolution(ADCres);

	/*Inputs*/
	pinMode(ignicao, INPUT_PULLUP);
	pinMode(oilpressure, INPUT_PULLUP);
	pinMode(navigationButton, INPUT_PULLUP);
	pinMode(chronoButtonStart, INPUT_PULLUP);
	pinMode(chronoButtonReset, INPUT_PULLUP);
	pinMode(waterTemperatureSensor, INPUT_PULLUP);
	// pinMode(rpmSensor, INPUT_PULLUP);
	pinMode(interruptPin, INPUT_PULLUP);
	pinMode(tensaoBateria, INPUT_PULLUP);
	pinMode(NivelComb, INPUT_PULLUP);

	/*Interrupcoes para botoes*/
	attachInterrupt(digitalPinToInterrupt(navigationButton), menuShift, RISING);
	attachInterrupt(digitalPinToInterrupt(chronoButtonStart), &chronoStartStop,RISING);
	attachInterrupt(digitalPinToInterrupt(chronoButtonReset), chronoReset,RISING);
	attachInterrupt(digitalPinToInterrupt(interruptPin),&vExampleInterruptHandler, RISING);

	/*Criacao de Queues*/


	waterTempQueue = xQueueCreate(1, sizeof(int));
	chronoToDisplayQueue = xQueueCreate(1, sizeof(chronoToDisplay));
	oilpressureQueue = xQueueCreate(1, sizeof(bool));
	rpmQueue = xQueueCreate(1, sizeof(int));
	ignitionQueue = xQueueCreate(1, sizeof(bool));
	BatvoltageQueue = xQueueCreate(1, sizeof(int));
	CombustivelQueue = xQueueCreate(1, sizeof(int));

	/*Criacao de tarefas*/
	xTaskCreatePinnedToCore(vMenuShift, "Menu button", 800, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(vChronoStartStop, "Chrono button", 2048, NULL, 5,NULL, 1);
	xTaskCreatePinnedToCore(vChronoReset, "Reset Button", 2048, NULL, 3, NULL,0);
	xTaskCreatePinnedToCore(vTimeKeeping, "Chronometer", 4096, NULL, 2, NULL,0);
	xTaskCreatePinnedToCore(vWaterTemperature, "Water Temperature", 1024, NULL,3, NULL, 0);
	xTaskCreatePinnedToCore(vDisplay1, "Display1", 4096, NULL, 4, NULL, 1);
	// xTaskCreatePinnedToCore(vRpm, "rpm", 1024, NULL, 3, NULL, 0);  //com pot
	xTaskCreatePinnedToCore(vIgnition, "ignition state", 1024, NULL, 1, NULL,0);
	xTaskCreatePinnedToCore(vTensaoBateria, "tensao da bateria", 1024, NULL, 3,NULL, 0);
	xTaskCreatePinnedToCore(vCombustivel, "nivel de combustivel", 1024, NULL, 3,NULL, 0);
	xTaskCreatePinnedToCore(vOilpressure, "pressao oleo", 1024, NULL, 3, NULL,0);
	if (xCountingSemaphore != NULL) {
		xTaskCreatePinnedToCore(vHandlerTask, "Handler", 1024, NULL, 5, NULL,1); //rpm com freq
	}
	/*Inicializacao da porta serie*/

	Serial.begin(115200);
	while (!Serial)
		;
	Serial2.begin(9600, SERIAL_8N1, 16, 17);
	while (!Serial2)
		;

}

void loop() {

	vTaskDelete(NULL);  //matar a tarefa arduino
}

void vDisplay1(void *pvParameters) {

	short int waterTemperature = 0, waterTemperature_tmp = 0;
	short int miliseconds = 0;
	short int seconds = 0;
	short int minutes = 0;
	short int hours = 0;
	short int rpm = 0, rpm_tmp;
	short int fuel = 0, fuel_tmp = 0;
	int voltage = 125, voltage_tmp = 125;
	bool pressure = 0, pressure_tmp = 0;
	bool ignition_state = 0, ignition_state_tmp = 0;

	TickType_t xLastWakeTime = xTaskGetTickCount();
	chronoToDisplay receiveFromChrono;

	for (;;) {

		if (xQueueReceive(waterTempQueue, &waterTemperature_tmp, 0) == pdTRUE) {
			waterTemperature = waterTemperature_tmp;
		}
		if ( xQueueReceive(oilpressureQueue, &pressure_tmp, 0) == pdTRUE) {
			pressure = pressure_tmp;
		}
		if (xQueueReceive(ignitionQueue, &ignition_state_tmp, 0) == pdTRUE) {
			ignition_state = ignition_state_tmp;
		}
		if (xQueuePeek(rpmQueue, &rpm_tmp, 0) == pdTRUE) {
			rpm = rpm_tmp;
		}
		if (xQueuePeek(BatvoltageQueue, &voltage_tmp, 0) == pdTRUE) {
			voltage = voltage_tmp;
		}
		if (xQueueReceive(CombustivelQueue, &fuel_tmp, 0) == pdTRUE) {
			fuel = fuel_tmp;
		}
		xQueuePeek(chronoToDisplayQueue, &receiveFromChrono, 0);
		miliseconds = receiveFromChrono.tenthOfASeconds;
		seconds = receiveFromChrono.seconds;
		minutes = receiveFromChrono.minutes;
		hours = receiveFromChrono.hours;

		Serial.print("* fuel=");
		Serial.println(fuel);
		Serial.print("* waterTemperature=");
		Serial.println(waterTemperature);
		Serial.print("* voltage=");
		Serial.println(voltage);

		Serial2.print("main.rpm.val=");
		Serial2.print(rpm);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.fuel.val=");
		Serial2.print(fuel);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.water_temp.val=");
		Serial2.print(waterTemperature);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.miliseconds.val=");
		Serial2.print(miliseconds);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.seconds.val=");
		Serial2.print(seconds);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.minutes.val=");
		Serial2.print(minutes);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.hours.val=");
		Serial2.print(hours);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.pressure.val=");
		Serial2.print(pressure);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.ignition_state.val=");
		Serial2.print(ignition_state);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		Serial2.print("main.voltage.val=");
		Serial2.print(voltage);

		Serial2.write(0xff);
		Serial2.write(0xff);
		Serial2.write(0xff);

		/* if (waterTemperature <= 85) {


		 Serial2.print("main.water_temp.val=");
		 Serial2.print(waterTemperature);
		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 }
		 if (pressure == 0) {

		 Serial2.print("main.pressure.val=");
		 Serial2.print(pressure);
		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 }

		 if (voltage >= 125) {


		 Serial2.print("main.voltage.val=");
		 Serial2.print(voltage);

		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 Serial2.write(0xff);
		 }*/
	}
	vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
}

/*void vRpm(void *pvParameters) {
 TickType_t xLastWakeTime = xTaskGetTickCount();
 short int rpm = 0, rpmValue = 0;

 for (;;) {
 rpmValue = analogRead(rpmSensor);
 //  Serial.print("Valor de rpm: ");
 //Serial.println(rpmValue);
 rpm = rpmValue * (maxrpm - minrpm) / (pow(2.0, (float)ADCres)) + minrpm;
 /* Serial.print("rpm: ");
 Serial.println(rpm);

 xQueueOverwrite(rpmQueue, &rpm);
 vTaskDelayUntil(&xLastWakeTime, (rpmPeriod / portTICK_PERIOD_MS));
 }
 }*/

void vWaterTemperature(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int waterTemperature = 0, sensorValue = 0;

	for (;;) {
		sensorValue = analogRead(waterTemperatureSensor);
		//  Serial.print("Valor de sensor: ");
		//Serial.println(sensorValue);
		waterTemperature = sensorValue * (maxWaterTemp - minWaterTemp)
				/ (pow(2.0, (float) ADCres)) + minWaterTemp;

		Serial.print("Temperatura de agua: ");
		Serial.println(waterTemperature);

		xQueueOverwrite(waterTempQueue, &waterTemperature);
		vTaskDelayUntil(&xLastWakeTime,
				(WaterTempReadPeriod / portTICK_PERIOD_MS));
	}
}

void vMenuShift(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int option = 0;
	bool display = 0;
	unsigned long time1 = 0, time2 = 0;

	// xSemaphoreTake(menuSemaphore, 0);
	for (;;) {
		xSemaphoreTake(menuSemaphore, portMAX_DELAY);
		//Redundancia
		time2 = millis();
		if ((time2 - time1) > DEBOUNCE_TIME) {
			time1 = time2;

			option++;

			if (option > 2) {
				option = 1;
			}

			switch (option) {

			case 1:

				Serial2.print("page 0");

				Serial2.write(0xff);
				Serial2.write(0xff);
				Serial2.write(0xff);
				 Serial.println("*************");
				break;

			case 2:
				Serial2.print("page 4");

				Serial2.write(0xff);
				Serial2.write(0xff);
				Serial2.write(0xff);
				 Serial.println("!!!!!!!!!!!!!!!!");
				break;

			}

		}

	}

}

void  vChronoStartStop(void *pvParameters) {
	unsigned long time1 = 0, time2 = 0;
	for (;;) {
		xSemaphoreTake(chrono_Start_StopSemaphore, portMAX_DELAY); // Espera até receber o semáforo do botão de iniciar/parar
		 Serial.println("*************");
		//Redundancia
		time2 = millis();
		if ((time2 - time1) > DEBOUNCE_TIME) {
			time1 = time2;
		}
		if (!started) {  // Se o cronómetro ainda não foi iniciado
			start_time = millis();  // Guarda o tempo inicial
			running = true;  // Indica que o cronómetro está a correr
			started = true;  // Indica que o cronómetro já foi iniciado
		} else {
			if (running) {  // Se o cronómetro estiver a correr
				current_time = millis() - start_time;  // Calcula o tempo atual
				running = false; //pare o cronometro
			} else {
				start_time = millis() - current_time; //retome a contagem de onde parou
				running = true; //inicia o cronometro novamente
			}
		}
	}
}

void vChronoReset(void *pvParameters) {
	unsigned long time1 = 0, time2;
	for (;;) {
		xSemaphoreTake(chrono_ResetSemaphore, portMAX_DELAY); // Espera até receber o semáforo do botão de reset
		//Redundancia
		time2 = millis();
		if ((time2 - time1) > DEBOUNCE_TIME) {
			time1 = time2;
		}
		start_time = 0;  // Reinicializa o tempo inicial do cronómetro
		current_time = 0;  // Reinicializa o tempo atual do cronómetro
		running = false;  // Indica que o cronómetro não está a correr
		started = false;  // Indica que o cronómetro ainda não foi iniciado
		//Adicionado
		int miliseconds = 0;  // Reinicializa os décimos de segundo
		int seconds = 0;  // Reinicializa os segundos
		int minutes = 0;  // Reinicializa os minutos
		int hours = 0;  // Reinicializa as horas
	}
}

void vTimeKeeping(void *pvParameters) {
	chronoToDisplay sendFromChrono;
	for (;;) {
		if (running) { // Se o cronómetro estiver a correr
			unsigned long current_time_aux = millis(); // Guarda o tempo atual
			current_time = current_time_aux - start_time; // Calcula o tempo atual
			int miliseconds = current_time / 100; // Calcula os décimos de segundo
			int seconds = miliseconds / 10; // Calcula os segundos
			int minutes = seconds / 60; // Calcula os minutos
			int hours = minutes / 60; // Calcula as horas
			miliseconds %= 10; // Calcula os décimos de segundo restantes
			seconds %= 60; // Calcula os segundos restantes
			minutes %= 60; // Calcula os minutos restantes
			hours %= 24; // Calcula as horas

			Serial.print(hours);
			Serial.print(":");
			Serial.print(minutes);
			Serial.print(":");
			Serial.print(seconds);
			Serial.print(":");
			Serial.println(miliseconds);
			Serial.print("Tempo:  ");

			sendFromChrono.tenthOfASeconds = miliseconds;
			sendFromChrono.seconds = seconds;
			sendFromChrono.minutes = minutes;
			sendFromChrono.hours = hours;

			xQueueOverwrite(chronoToDisplayQueue, &sendFromChrono);

		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
void vOilpressure(void *pvParamenters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();

	bool pressure = 0;

	for (;;) {

		//Polling

		pressure = digitalRead(oilpressure); //À variuavel de pressao é atribuido um valor logico, HIGH(1) para pressao de oleo, LOW(0) para sem pressao de oleo

		if (pressure == LOW) {
			pressure = 1;

		} else {
			pressure = 0;
		}

		xQueueSendToBack(oilpressureQueue, &pressure, 0);
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
	}
}

void vIgnition(void *pvParamenters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();

	bool ignition_state = 0;

	for (;;) {

		//Polling

		ignition_state = digitalRead(ignicao);

		if (ignition_state == HIGH) {
			ignition_state = 0;
			//ignicao on
		} else {
			ignition_state = 1;
		}

		xQueueSendToBack(ignitionQueue, &ignition_state, 0);
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
	}
}

void vTensaoBateria(void *pvParamenters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int tensaobateriaValue = 0;
	int voltage = 0;
	for (;;) {
		//  Serial.println("estou aquii ");
		tensaobateriaValue = analogRead(tensaoBateria);
		//  Serial.print("Valor de sensor: ");
		//Serial.println(sensorValue);
		voltage = (int) (tensaobateriaValue * (tensaomax - tensaomin)
				/ (pow(2.0, (float) ADCres)) + tensaomin);

		/* Serial.print("voltage: ");
		 Serial.println(voltage);*/

		// xQueueSendToBack(BatvoltageQueue, &voltage, 0);
		xQueueOverwrite(BatvoltageQueue, &voltage);

		vTaskDelayUntil(&xLastWakeTime, (voltagePeriod / portTICK_PERIOD_MS));
	}
}

void vCombustivel(void *pvParamenters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int combustivelValue = 0;
	int fuel = 0;
	for (;;) {
		// Serial.println("estou aquii ");
		combustivelValue = analogRead(NivelComb);

		fuel = combustivelValue * (combmax - combmin)/ (pow(2.0, (float) ADCres)) + combmin;
		Serial.println("estou aquii ");
		Serial.print(" litros: ");
		Serial.println(fuel);

		xQueueSendToBack(CombustivelQueue, &fuel, 0);
		vTaskDelayUntil(&xLastWakeTime, (FUELPERIOD / portTICK_PERIOD_MS));
	}
}

static void vHandlerTask(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	long int count_pulses = 0;
	long int pulses_per_second = 0;
	int rpm = 0;
	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {

		while ( xSemaphoreTake(xCountingSemaphore, 0 ) == pdTRUE) {

			count_pulses++;
		}
		/* To get here the event must have occurred.  Process the event (in this
		 case we just print out a message). */

		pulses_per_second = count_pulses * 2;

		rpm = pulses_per_second * 60;
//Serial.println("*************");
		/*Serial.print("rpm:");
		 Serial.println(rpm);
		 /*
		 Serial.print(" Pulses_per_sec:");
		 Serial.println(pulses_per_second);
		 Serial.println("*************");*/
		count_pulses = 0;

		xQueueOverwrite(rpmQueue, &rpm);
		vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));

	}
}

/* Inteerrupcoes*/

void menuShift() {
xSemaphoreGiveFromISR(menuSemaphore, NULL);
	//static signed portBASE_TYPE xHigherPriorityTaskWoken;
	//	xSemaphoreGiveFromISR(menuSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);
}
void chronoStartStop() {
	//static signed portBASE_TYPE xHigherPriorityTaskWoken;
	//xSemaphoreGiveFromISR(chrono_Start_StopSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken); // Envia o semáforo para a tarefa de tratamento do botão de iniciar/parar
	xSemaphoreGiveFromISR(chrono_Start_StopSemaphore,NULL);
	//Serial.println("*************");
}

void  chronoReset() {
	xSemaphoreGiveFromISR(chrono_ResetSemaphore, NULL); // Envia o semáforo para a tarefa de tratamento do botão de iniciar/parar
}

static void IRAM_ATTR vExampleInterruptHandler() {
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(xCountingSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);
}
