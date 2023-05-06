//------Includes
#include "Arduino.h"
#include "soniclib.h" //function declarations in ch_api.c
#include "chirp_bsp.h"
#include "teensyChirpPins.h"
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "app_config.h"


void setup() {
  Serial.begin(9600);
  // pin mode defines
  pinMode(ledPin, OUTPUT);
  pinMode(progPin, OUTPUT);
  digitalWrite(progPin, LOW);
  pinMode(rstPin, OUTPUT);
  digitalWrite(rstPin, HIGH);
  
  Wire.begin();

  //defining bsp file
  //Chirp sensor configuration structure (ch_group_t from sonicLib.h)
  struct ch_dev_t *chirp1;
  chirp1 = (ch_dev_t*)malloc(sizeof(ch_dev_t));
  struct ch_group_t *chGroup1;
  chGroup1 = (ch_group_t*)malloc(sizeof(chGroup1));
  ch_config_t *chirp1Config;
  chirp1Config = (ch_config_t*)malloc(sizeof(chirp1Config));
  
  // 1 initializing hardware
  chbsp_board_init(chGroup1);
  // 2 initialize sonicLib structures for each sensor (only 1 for now). 
  //ch_init(chirp1, chGroup1, 1, CHIRP_SENSOR_FW_INIT_FUNC);
  // 3 interrupt handler function () here. //uses ch_get_range() to read 
  // 4 program and start all sensors
  //ch_group_start(chGroup1);
  // 5 configure sensors
  //ch_set_config(chirp1, chirp1Config);
}

void loop() {

}
