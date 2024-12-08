#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define STEP_pin 28
#define DIR_pin 27
#define MICROSTEPS 8
#define DELAY_ms 10
#define STEP_LENGTH_ms 1

typedef struct{
   int en_pin;
   int home_pin;
   int zero_offset;
   int position;
} axis;

axis axes[12];

bool homing_commanded = 0;
bool move_commanded = 0;
int active_axis = 0;
int destination = 0;

int calculate_steps_from_switch_pos(int switch_pos){
  return (200*MICROSTEPS/12)*switch_pos;
}

int round_to_fullsteps(int numToRound){
  // We disable drivers when not in use, causing motors to snap to the nearest full step.
  // Doing this often accumulates a position error, so we should only halt at full steps.
  int remainder = numToRound % (MICROSTEPS*4);
  if (remainder == 0)
    return numToRound;
  if (remainder >= MICROSTEPS*2)
    return numToRound + MICROSTEPS*4 - remainder;
  else
    return numToRound - remainder;
}

void update_dir(){
  gpio_put(DIR_pin, (destination < axes[active_axis].position));
  sleep_ms(DELAY_ms);
}

void process_command(char *command){
  printf(command);
  printf(":");
  if (command[0] > 96 && command[0] < 109){
    if (command[0]-97 != active_axis){
      active_axis = command[0]-97;
      printf("change driver to ");
      printf("%d",command[0]-97);
      printf(":");
    }
    if (command[1]=='0'){
      // If commanded destination is 0 we go home
      destination=-1;
      update_dir();
      gpio_put(axes[active_axis].en_pin, 0);
      sleep_ms(DELAY_ms);

      printf("homing commanded:");
      homing_commanded = 1;
      return;
    }
    if(command[1] > 48 && command[1] < 60){
      // Otherwise we go to destination
      // Switch positions 123456789:;
      destination=calculate_steps_from_switch_pos(command[1]%12);
      destination-=axes[active_axis].zero_offset;
      destination=round_to_fullsteps(destination);

      if (destination == axes[active_axis].position){
        printf("done\n");
        return;
      }

      update_dir();
      gpio_put(axes[active_axis].en_pin, 0);
      sleep_ms(DELAY_ms);
      move_commanded = 1;
      printf("dest ");
      printf("%d",destination);
      printf(":");
      printf("pos ");
      printf("%d",axes[active_axis].position);
      printf(":");
    }
    else
      printf("position out of range 0 ... ;\n");
  return;
  }
  else
    printf("axis out of range a ... l\n");
  return;
}

void serial_available(void* cmd){
  // receive characters from serial
  volatile signed char *c = (signed char *) cmd;
  while ((*c = getchar_timeout_us(0)) > 0)
      c++;
  *c = '\0';
}

void wiggle(){
  int last=0;
  for(int i=6;i>=0;i--){
    gpio_put(DIR_pin, (i%2 == 0));
    sleep_ms(DELAY_ms);
    int j=(i+last)*MICROSTEPS;
    last = i;
    while(j){
      gpio_put(STEP_pin, 1);
      sleep_ms(STEP_LENGTH_ms);
      gpio_put(STEP_pin, 0);
      sleep_ms(STEP_LENGTH_ms);
      j--;
      if (!gpio_get(DIR_pin))
        axes[active_axis].position++;
      else
        axes[active_axis].position--;
    }
  }
}

void move_worker(){
  gpio_put(STEP_pin, 1);
  sleep_ms(STEP_LENGTH_ms);
  gpio_put(STEP_pin, 0);
  sleep_ms(STEP_LENGTH_ms);
  if (!gpio_get(DIR_pin))
    axes[active_axis].position++;
  else
    axes[active_axis].position--;

  if (destination == axes[active_axis].position){
    move_commanded=0;
    sleep_ms(DELAY_ms);
    wiggle();
    gpio_put(axes[active_axis].en_pin, 1);
    sleep_ms(DELAY_ms);
    printf("done\n");
  }
}

void home_worker(){
  if (gpio_get(axes[active_axis].home_pin)){
    // Snap to nearest full step
    gpio_put(axes[active_axis].en_pin, 1);
    sleep_ms(DELAY_ms);
    gpio_put(axes[active_axis].en_pin, 0);
    sleep_ms(DELAY_ms);
    axes[active_axis].position=0;
    destination=calculate_steps_from_switch_pos(1);
    destination-=axes[active_axis].zero_offset;
    destination=round_to_fullsteps(destination);
    update_dir();
    move_commanded = 1;
    homing_commanded = 0;
  }
  else{
    gpio_put(STEP_pin, 1);
    sleep_ms(STEP_LENGTH_ms*2);
    gpio_put(STEP_pin, 0);
    sleep_ms(STEP_LENGTH_ms*2);
  }
}

int main(){

  axes[0].en_pin=26;
  axes[0].home_pin=9;
  axes[0].zero_offset=455; //a
  axes[0].position=0;

  axes[1].en_pin=16;
  axes[1].home_pin=5;
  axes[1].zero_offset=0; //b
  axes[1].position=0;

  axes[2].en_pin=22;
  axes[2].home_pin=10;
  axes[2].zero_offset=520; //c
  axes[2].position=0;

  axes[3].en_pin=17;
  axes[3].home_pin=4;
  axes[3].zero_offset=0; //d
  axes[3].position=0;

  axes[4].en_pin=21;
  axes[4].home_pin=11;
  axes[4].zero_offset=230; //e
  axes[4].position=0;

  axes[5].en_pin=12;
  axes[5].home_pin=3;
  axes[5].zero_offset=0; //f
  axes[5].position=0;

  axes[6].en_pin=20;
  axes[6].home_pin=8;
  axes[6].zero_offset=65; //g
  axes[6].position=0;

  axes[7].en_pin=13;
  axes[7].home_pin=2;
  axes[7].zero_offset=0; //h
  axes[7].position=0;

  axes[8].en_pin=19;
  axes[8].home_pin=7;
  axes[8].zero_offset=335; //i
  axes[8].position=0;

  axes[9].en_pin=14;
  axes[9].home_pin=1;
  axes[9].zero_offset=0; //j
  axes[9].position=0;

  axes[10].en_pin=18;
  axes[10].home_pin=6;
  axes[10].zero_offset=1005; //k
  axes[10].position=0;

  axes[11].en_pin=15;
  axes[11].home_pin=0;
  axes[11].zero_offset=0; //l
  axes[11].position=0;

  for (int i; i<12; i++){
    gpio_init(axes[i].en_pin);
    gpio_set_dir(axes[i].en_pin, GPIO_OUT);
    gpio_put(axes[i].en_pin, 1);
    gpio_init(axes[i].home_pin);
    gpio_set_dir(axes[i].home_pin, GPIO_IN);
  }

  gpio_init(STEP_pin);
  gpio_set_dir(STEP_pin, GPIO_OUT);
  gpio_init(DIR_pin);
  gpio_set_dir(DIR_pin, GPIO_OUT);

  stdio_init_all();
  sleep_ms(1000);

  printf("Here goes nothin ...\n");

  // TODO Crash if longer strings are received?
  // TODO Terrible things happen if we interrupt an operation with another cmd
  static signed char serialString[32] = {0};
  stdio_set_chars_available_callback(&serial_available, (void*) serialString);

  while (1) {
    if (serialString[0]){
      process_command(serialString);
      serialString[0] = '\0';
    }
    if (move_commanded)
      move_worker();
    if(homing_commanded)
      home_worker();
  }
}