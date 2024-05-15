#include <AccelStepper.h>

#define STEP 22
#define DIR 24
#define HOME 26
#define MAXSPEED 500
#define ACCELERATION 5000
#define MICROSTEPS 8

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

typedef struct{
   int en_pin;
   long position=0;
} axis;

axis axes[12];

boolean homing_commanded = 0;
boolean move_commanded = 0;
int active_axis = 0;
long destination = 0;


long calculate_steps_from_switch_pos(int switch_pos){
    return (200*MICROSTEPS/12)*switch_pos;
}


void process_command(String command){
  Serial.print(command);
  Serial.print(":");
  if(command.length()>4||command.length()<2)
    return;
  if (isAlpha(command[0])&&command[0]<'m'){
    if (tolower(command[0])-97 != active_axis){
      active_axis = tolower(command[0])-97;
      stepper.setCurrentPosition(axes[active_axis].position);
      Serial.print("changedriverto ");
      Serial.print(tolower(command[0])-97);
      Serial.print(":");
    }
    if (!(destination=command.substring(1).toInt()%12)){
      homing_commanded = 1;
      stepper.setSpeed(MAXSPEED*-0.5);
      return;
    }
    else
      move_commanded = 1;
      Serial.print("dest ");
      Serial.print(calculate_steps_from_switch_pos(destination));
      Serial.print(":");
      stepper.moveTo(calculate_steps_from_switch_pos(destination));
  }
  return;
}




void setup()
{
  axes[0].en_pin=28;
  axes[1].en_pin=30;
  axes[2].en_pin=32;
  axes[3].en_pin=34;
  axes[4].en_pin=42;
  axes[5].en_pin=40;

  axes[6].en_pin=40;
  axes[7].en_pin=42;
  axes[8].en_pin=44;
  axes[9].en_pin=46;
  axes[10].en_pin=48;
  axes[11].en_pin=50;

  pinMode(axes[0].en_pin, OUTPUT);
  digitalWrite(axes[0].en_pin, HIGH);
  pinMode(axes[1].en_pin, OUTPUT);
  digitalWrite(axes[1].en_pin, HIGH);
  pinMode(axes[2].en_pin, OUTPUT);
  digitalWrite(axes[2].en_pin, HIGH);
  pinMode(axes[3].en_pin, OUTPUT);
  digitalWrite(axes[3].en_pin, HIGH);
  pinMode(axes[4].en_pin, OUTPUT);
  digitalWrite(axes[4].en_pin, HIGH);
  pinMode(axes[5].en_pin, OUTPUT);
  digitalWrite(axes[5].en_pin, HIGH);

  pinMode(HOME, INPUT_PULLUP);

  stepper.setMaxSpeed(MAXSPEED);
  stepper.setAcceleration(ACCELERATION);

  Serial.begin(115200);
  while(!Serial);
}



void loop()
{

  if(homing_commanded){
    if (!digitalRead(HOME)){
      stepper.stop();
      stepper.setCurrentPosition(0);
      stepper.runToNewPosition(30);
      digitalWrite(axes[active_axis].en_pin, HIGH);
      axes[active_axis].position=0;
      stepper.setCurrentPosition(0);
      homing_commanded = 0;
      Serial.println("homed");
      return;
    }
    else{
      digitalWrite(axes[active_axis].en_pin, LOW);
      //Serial.println("homing");
      stepper.runSpeed();
      return;
    }
  }


  if(move_commanded){
    if (!stepper.distanceToGo()){
      digitalWrite(axes[active_axis].en_pin, HIGH);
      move_commanded = 0;
      axes[active_axis].position = stepper.currentPosition();
      Serial.print("reported pos ");
      Serial.print(stepper.currentPosition());
      Serial.print(":");
      Serial.println("done");
      return;
    }
    else{
      digitalWrite(axes[active_axis].en_pin, LOW);
      stepper.run();
      return;
    }
  }


  if (Serial.available())
    process_command(Serial.readStringUntil('\n'));

}