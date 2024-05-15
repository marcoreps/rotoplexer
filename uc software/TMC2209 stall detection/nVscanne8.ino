#include <TMCStepper.h>

#define BLANK_TIME 24
#define RMS_CURRENT 100
#define MICROSTEPS 8
#define COOLTHRS 0xFFFFF
#define SEMIN 0
#define SEDN 0b01
#define VELOCITY 1000
#define SG_DETECTION 80
#define PRE_HOME_RETRACT 200
#define POST_HOME_RETRACT 200

typedef struct{
   int rx;
   int tx;
   int position=42;
   TMC2209Stepper *driver;
} axis;

axis axes[12];

int initialized = 0;
boolean homing_commanded = 0;
boolean move_commanded = 0;
boolean homing_initiated = 0;
boolean change_driver = 1;
boolean endstop_found = 0;
int active_axis = 0;
int destination = 0;
TMC2209Stepper *driver;



void wiggle(){
  const int amplitude = 15;
  int repetitions = 15;
  driver->toff(4);
  while (repetitions){
    repetitions--;

    driver->shaft(!driver->shaft());
    delay(amplitude);
    driver->VACTUAL(VELOCITY);
    delay(amplitude);
    driver->VACTUAL(0);

    driver->shaft(!driver->shaft());
    delay(amplitude);
    driver->VACTUAL(VELOCITY);
    delay(amplitude);
    driver->VACTUAL(0);

    driver->shaft(!driver->shaft());
    delay(amplitude);
    driver->VACTUAL(VELOCITY);
    delay(amplitude);
    driver->VACTUAL(0);
  }
  delay(amplitude);
  driver->toff(0);
}


int calculate_direction(){
  if (destination>axes[active_axis].position)
    return 0;
  else
    return 1;
}

int calculate_traveltime(){
  destination-axes[active_axis].position;
    return abs((destination - axes[active_axis].position)*184);
}

void process_command(String command){
  Serial.print(command);
  Serial.print(":");
  if(command.length()>4||command.length()<2)
    return;
  if (isAlpha(command[0])&&command[0]<'m'){
    if (tolower(command[0])-97 != active_axis){
      active_axis = tolower(command[0])-97;
      change_driver = 1;
      Serial.print("changedriverto ");
      Serial.print(tolower(command[0])-97);
      Serial.print(":");
    }
    if (!(destination=command.substring(1).toInt()%12)){
      homing_commanded = 1;
      return;
    }
    else if (axes[active_axis].position == command.substring(1).toInt()%12)
      Serial.println("doneski");
    else
      move_commanded = 1;
  }
  return;
}

void setup() {
  //only the following can be used for RX: 10, 11, 12, 13, (14), (15), (50), 51, (52), 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)
  axes[0].rx=13;
  axes[0].tx=10;
  axes[0].driver= new TMC2209Stepper(axes[0].rx, axes[0].tx, 0.11f, 0b00);
  axes[0].driver->begin();
  axes[1].rx=12;
  axes[1].tx=9;
  axes[1].driver= new TMC2209Stepper(axes[1].rx, axes[1].tx, 0.11f, 0b00);
  axes[1].driver->begin();
  axes[2].rx=11;
  axes[2].tx=8;
  axes[2].driver= new TMC2209Stepper(axes[2].rx, axes[2].tx, 0.11f, 0b00);
  axes[2].driver->begin();
  axes[3].rx=62;
  axes[3].tx=64;
  axes[3].driver= new TMC2209Stepper(axes[3].rx, axes[3].tx, 0.11f, 0b00);
  axes[3].driver->begin();
  axes[4].rx=63;
  axes[4].tx=65;
  axes[4].driver= new TMC2209Stepper(axes[4].rx, axes[4].tx, 0.11f, 0b00);
  axes[4].driver->begin();
  axes[5].rx=66;
  axes[5].tx=67;
  axes[5].driver= new TMC2209Stepper(axes[5].rx, axes[5].tx, 0.11f, 0b00);
  axes[5].driver->begin();

  Serial.begin(115200);
  while(!Serial);


}

void loop() {



  if (change_driver){

    driver = axes[active_axis].driver;

    change_driver=0;
    return;
  }

  static long last_time=0;
  long ms = millis();
  if((ms-last_time) > 10) {
    last_time = ms;
    if (homing_initiated)
      if (driver->SG_RESULT()<SG_DETECTION){
        driver->VACTUAL(0);
        delay(300);
        driver->toff(0);
        delay(300);
        driver->shaft(0);
        delay(POST_HOME_RETRACT);
        homing_initiated = 0;
        endstop_found = 1;
        return;
      }
  }

  if(endstop_found){
    driver->toff(4);
    delay(300);
    driver->VACTUAL(VELOCITY/8);
    delay(POST_HOME_RETRACT);
    driver->VACTUAL(0);
    delay(POST_HOME_RETRACT);
    driver->toff(0);
    axes[active_axis].position=0;
    endstop_found = 0;
    homing_initiated = 0;
    Serial.println("homed");
    return;
  }

  if(homing_commanded){
        driver->toff(4);
        driver->shaft(1);
        driver->blank_time(BLANK_TIME);
        driver->rms_current(RMS_CURRENT);
        driver->I_scale_analog(0);
        //driver->ihold(0);
        driver->microsteps(MICROSTEPS);
        driver->semin(SEMIN);
        driver->sedn(SEDN);
        driver->VACTUAL(VELOCITY*-1);
        delay(PRE_HOME_RETRACT);
        driver->VACTUAL(VELOCITY);
        homing_commanded=0;
        homing_initiated=1;
        last_time=millis()+30;
        return;
    }



  if (initialized<6 && !homing_initiated){
    active_axis=initialized;
    change_driver=1;
    homing_commanded=1;
    initialized++;
    return;
  }


  if(move_commanded && !homing_initiated && axes[active_axis].position<12){
      driver->toff(4);
      driver->blank_time(BLANK_TIME);
      driver->rms_current(RMS_CURRENT);
      driver->I_scale_analog(0);
      //driver->ihold(0);
      driver->microsteps(MICROSTEPS);
      driver->semin(SEMIN);
      driver->sedn(SEDN);
      driver->shaft(calculate_direction());
      driver->VACTUAL(VELOCITY);
      delay(calculate_traveltime());
      driver->VACTUAL(0);
      driver->toff(0);
      axes[active_axis].position=destination;
      move_commanded=0;
      //wiggle();
      Serial.println("done");
      return;
    }


  if (Serial.available())
    process_command(Serial.readStringUntil('\n'));

}

