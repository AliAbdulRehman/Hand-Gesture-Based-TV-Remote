/*
   NRF24L01       Arduino_ Nano
   _____________________________
   VCC        |       3.3v
   GND        |       GND
   CSN        |   Pin10 SPI/SS
   CE         |   Pin9
   SCK        |   Pin13
   MOSI       |   Pin11
   MISO       |   Pin12
*/

//------------------------------------------------------------------------------------------------------------------
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//------------------------------------------------------------------------------------------------------------------
RF24 radio(9, 10); // CE, CSN
const byte address[5] = {'A', 'C', 'C', 'R', 'X'}; // address to communicate between RF modules
//------------------------------------------------------------------------------------------------------------------
//defining pins of ultrasonic sensors
#define trig_left 3
#define echo_left 2
#define trig_right 5
#define echo_right 4
//------------------------------------------------------------------------------------------------------------------
typedef enum {
  no_com,//no command being sent
  power,//power on/off
  vup,//volume up
  vdown,//volume down
  chnxt,//goto next channel
  chprv//goto previous channel
} commandTypedef;
//------------------------------------------------------------------------------------------------------------------
struct distance { //struct for distances of both sensors
  int left;
  int right;
};
//------------------------------------------------------------------------------------------------------------------
//Functionn Prototypes
void TaskReadSensorData( void *pvParameters );
void TaskProcessData( void *pvParameters );
void TaskSendData(void *pvParameters );
int calculate_distance(int trigger, int echo);
void pinConfig(void);
void radioSetup(void);
//------------------------------------------------------------------------------------------------------------------
SemaphoreHandle_t xSerialSemaphore; // Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
SemaphoreHandle_t xRadioSemaphore; // Declare a mutex Semaphore Handle which we will use to manage the Radio Port.
QueueHandle_t distanceQueue; // declare a Handle for Queue to store distance of both sensors (struct queue)
QueueHandle_t commandQueue; // declare a Handle for Queue to store command to be sent through radio (int queue)
//------------------------------------------------------------------------------------------------------------------
void setup() {
  //pinMode(button_pin, INPUT);
  Serial.begin(9600);
  pinConfig();
  radioSetup();
  distanceQueue = xQueueCreate(2, sizeof(struct distance));
  commandQueue = xQueueCreate(10, sizeof(int));
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  //------------------------------------------------------------------------------------------------------------------
  if ( xRadioSemaphore == NULL )  // Check to confirm that the Radio Semaphore has not already been created.
  {
    xRadioSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Radio Port
    if ( ( xRadioSemaphore ) != NULL )
      xSemaphoreGive( ( xRadioSemaphore ) );  // Make the Radio Port available for use, by "Giving" the Semaphore.
  }
  //------------------------------------------------------------------------------------------------------------------
  if (distanceQueue != NULL) {
    xTaskCreate(
      TaskReadSensorData
      ,  "Get Sensor Data"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

    xTaskCreate(
      TaskProcessData
      ,  "process sensor data"
      ,  128  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );

    xTaskCreate(
      TaskSendData
      ,  "send sensor data"
      ,  128  // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL );
  }
  // start scheduler
  Serial.println("starting application");
  vTaskStartScheduler();
  //------------------------------------------------------------------------------------------------------------------
  //OS should not allow program to get past this line
  Serial.println("Insufficient RAM");//in case of insufficient RAM, print error
  while (1);
}
//------------------------------------------------------------------------------------------------------------------
void loop() {
  // Empty. Things are done in Tasks.
}
//------------------------------------------------------------------------------------------------------------------
/*---------------------- Tasks ---------------------*/
//------------------------------------------------------------------------------------------------------------------
void TaskReadSensorData( void *pvParameters __attribute__((unused)) ) {
  for (;;) {
    struct distance dist;
    dist.right = calculate_distance(trig_right, echo_right);
    dist.left = calculate_distance(trig_left, echo_left);
    xQueueSend(distanceQueue, &dist, 5);
    //Serial.println("Calculating distance");
  }
}
//------------------------------------------------------------------------------------------------------------------
void TaskProcessData( void *pvParameters __attribute__((unused)) ) {
  struct distance prev_dist;
  struct distance current_dist;
  struct distance difference;
  int command;
  for (;;) {
    if (xQueueReceive(distanceQueue, &prev_dist, 5) == pdPASS) {
      if (xQueueReceive(distanceQueue, &current_dist, 5) == pdPASS) {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
          difference.left = current_dist.left - prev_dist.left;
          difference.right = current_dist.right - prev_dist.right;
          if ((current_dist.left >= 40 && current_dist.right >= 40) && (current_dist.left < 60 && current_dist.right < 60)) {
            //radio.write(power, sizeof(commandTypedef));
            Serial.println("turn power on/off");
            command = power;
          }
          else if (difference.left > 0) {
            //radio.write(vup, sizeof(commandTypedef));
            Serial.println("vup");
            command = vup;
          }
          else if (difference.left < 0) {
            //radio.write(vdown, sizeof(commandTypedef));
            Serial.println("vdown");
            command = vdown;
          }
          else if (difference.right > 0) {
            //radio.write(chnxt, sizeof(commandTypedef));
            Serial.println("chnxt");
            command = chnxt;
          }
          else if (difference.right < 0) {
            //radio.write(chprv, sizeof(commandTypedef));
            Serial.println("chprv");
            command = chprv;
          }
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
          xQueueSend(commandQueue, &command, 5);// send the command to queue for RF task
        }
      }
    }
  }
}
//------------------------------------------------------------------------------------------------------------------
void TaskSendData( void *pvParameters __attribute__((unused)) ) {
  int command;
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  for (;;) {
    if (xQueueReceive(commandQueue, &command, 50) == pdPASS) {
      if ( xSemaphoreTake( xRadioSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
        radio.write(&command, sizeof(command));
        if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 5) == pdTRUE) {
          Serial.print("Command sent : ");
          Serial.println(command);
          xSemaphoreGive(xSerialSemaphore);
        }
        xSemaphoreGive( xRadioSemaphore ); // Now free or "Give" the Radio Port for others.
      }
    }
    //vTaskDelay(xDelay);
  }
}
//------------------------------------------------------------------------------------------------------------------
//Function to calculate distance
int calculate_distance(int trigger, int echo)
{
  int dist;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);//sending pulse for 10 ms
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  int time_taken;
  time_taken = pulseIn(echo, HIGH);//storing time taken to receive echo
  dist = time_taken * 0.034 / 2;//calculating distance
  if (dist > 60)
    dist = 60; //max distance 60
  return dist;
}
//------------------------------------------------------------------------------------------------------------------
//function to configure pins
void pinConfig(void) {
  pinMode(trig_left, OUTPUT);
  pinMode(echo_left, INPUT);
  pinMode(trig_right, OUTPUT);
  pinMode(echo_right, INPUT);
}
//------------------------------------------------------------------------------------------------------------------
//function to setup radio
void radioSetup(void) {
  radio.begin();
  while (!radio.isChipConnected());
  Serial.println("Connected");
  radio.openWritingPipe(address); //setting the address of the receiver to which we will send data, the 5 letter string we previously set.
  radio.setAddressWidth(5); // Using 5 byte addresses
  radio.setPALevel(RF24_PA_MIN); //setting the Power Amplifier level, in our case we will set it to minimum as my modules are very close to each other.
  radio.stopListening(); //setting the module as transmitter
}
