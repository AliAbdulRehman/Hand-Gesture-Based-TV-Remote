/*
   NRF24L01    Blue_Pill(stm32f01C)
   __________________________________________________________________________
   VCC        |     3.3v
   GND        |      GND
   CSN        |     A4 NSS1 (PA4) 3.3v
   CE         |     B0 digital (PB0) 3.3v
   SCK        |     A5 SCK1   (PA5) 3.3v
   MOSI       |     A7 MOSI1  (PA7) 3.3v
   MISO       |     A6 MISO1  (PA6) 3.3v


*/
//------------------------------------------------------------------------------------------------------------------

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//------------------------------------------------------------------------------------------------------------------

typedef enum{
  no_com,//no command being sent
  power,//power on/off
  vup,//volume up
  vdown,//volume down
  chnxt,//goto next channel
  chprv//goto previous channel
}commandTypedef;

//------------------------------------------------------------------------------------------------------------------

// function prototype
void radioSetup(void);
void pinConfig(void);
RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const byte address[5] = {'A', 'C', 'C', 'R', 'X'}; // address to communicate between RF modules
boolean power_state = 0; // to be used for checking TV on/off state
int led_pin = PC13;  //set the led indicator to pin C13
uint8_t command = no_com; //to be used to store incoming command

//------------------------------------------------------------------------------------------------------------------

// setup
void setup() {
  Serial.begin(9600);
  pinConfig();
  radioSetup();
}

//------------------------------------------------------------------------------------------------------------------

void loop()
{
  if (radio.available())              //Looking for the data.
  {
    Serial.println("Reciever is active for recieving data");

    //Getting command from transmitter
    radio.read(&command, sizeof(command));    //Reading the data
    // the following line is just for testing purposes
    Serial.println(command);
    // using switch case
    switch ( command )
    {
      // power state 0 is TV is previously off
      // power state 1 is TV is on
      case (power): // power on/off
        if (power_state == 0 ) {
          Serial.println("Power ON!");
        }
        else {
          Serial.println("Power OFF!");
        }
        power_state = !power_state ; // toggling the power state
        break;
      case (vup): // volume up
        if ( power_state == 0 ) {
          Serial.println("COMMAND NOT EXCEPTED, SWITCH ON TV FIRST ");
        }
        else {
          Serial.println("Volume UP!");
        }
        break;
      case (vdown): // volume down
        if (power_state == 0 ) {
          Serial.println("COMMAND NOT EXCEPTED, SWITCH ON TV FIRST ");
        }
        else {
          Serial.println("Volume DOWN!");
        }
        break;
      case (chnxt): // channel next
        if (power_state == 0 ) {
          Serial.println("COMMAND NOT EXCEPTED, SWITCH ON TV FIRST ");
        }
        else {
          Serial.println("Next Channel!");
        }
        break;
      case (chprv): // previous channel
        if (power_state == 0 ) {
          Serial.println("COMMAND NOT EXCEPTED, SWITCH ON TV FIRST ");
        }
        else {
          Serial.println("Previous Channel!");
        }
        break;
      default :
        Serial.println("No command received");
        break;
    }
    //digitalWrite(led_pin, power_state);
  }
}
//------------------------------------------------------------------------------------------------------------------
// radio setup function
void radioSetup(void)
{
  radio.begin();
  while (!radio.isChipConnected());
  Serial.println("Connected");
  radio.openReadingPipe(0, address); //Setting the address of the transmitter from which we will receive data.
  radio.setPALevel(RF24_PA_MIN); //Setting the Power Amplifier level to minimum
  radio.startListening(); //Setting the module as receiver
}
//------------------------------------------------------------------------------------------------------------------
// pin configuration function
void pinConfig(void) {
  pinMode(led_pin, OUTPUT);
}
