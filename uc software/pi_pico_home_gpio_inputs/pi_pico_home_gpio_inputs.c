#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>

#define STEP_pin 28
#define DIR_pin 27
#define MICROSTEPS 8
#define DELAY_ms 100
#define STEP_LENGTH_ms 2

typedef struct{
   int en_pin;
   int home_pin;
   long position;
} axis;

axis axes[12];

bool homing_commanded = 0;
bool move_commanded = 0;
int active_axis = 0;
long destination = 0;

long calculate_steps_from_switch_pos(int switch_pos){
    return (2*200*MICROSTEPS/12)*switch_pos;
}

void update_dir(){
    gpio_put(DIR_pin, (destination >= axes[active_axis].position));
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
    if (!(destination=calculate_steps_from_switch_pos(command[1]%12))){
      // If commanded destination is 0 we go home
      gpio_put(axes[active_axis].en_pin, 0);
      sleep_ms(DELAY_ms);
      destination=-1;
      update_dir();
      printf("homing commanded:");
      homing_commanded = 1;
      return;
    }
    else{
      // Otherwise we go to destination
      gpio_put(axes[active_axis].en_pin, 0);
      sleep_ms(DELAY_ms);
      update_dir();
      move_commanded = 1;
      printf("dest ");
      printf("%d",destination);
      printf(":");
    }
  }
  else
    printf("axis out of range a ... l\n");
  return;
}

void serial_available(void* cmd)
{
    // receive characters from serial
    volatile signed char *c = (signed char *) cmd;
    while ((*c = getchar_timeout_us(0)) > 0)
        c++;
    *c = '\0';
}


int main(){

    axes[0].en_pin=26;
    axes[0].home_pin=9;
    axes[0].position=0;

    axes[1].en_pin=16;
    axes[1].home_pin=5;
    axes[1].position=0;

    axes[2].en_pin=22;
    axes[2].home_pin=10;
    axes[2].position=0;

    axes[3].en_pin=17;
    axes[3].home_pin=4;
    axes[3].position=0;

    axes[4].en_pin=21;
    axes[4].home_pin=11;
    axes[4].position=0;

    axes[5].en_pin=12;
    axes[5].home_pin=3;
    axes[5].position=0;

    axes[6].en_pin=20;
    axes[6].home_pin=8;
    axes[6].position=0;

    axes[7].en_pin=13;
    axes[7].home_pin=2;
    axes[7].position=0;

    axes[8].en_pin=19;
    axes[8].home_pin=7;
    axes[8].position=0;

    axes[9].en_pin=14;
    axes[9].home_pin=1;
    axes[9].position=0;

    axes[10].en_pin=18;
    axes[10].home_pin=6;
    axes[10].position=0;

    axes[11].en_pin=15;
    axes[11].home_pin=0;
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
    static signed char serialString[32] = {0};
    stdio_set_chars_available_callback(&serial_available, (void*) serialString);

    while (1) {
        if (serialString[0]){
          process_command(serialString);
          serialString[0] = '\0';
        }
        if (move_commanded){
          if (destination == axes[active_axis].position){
            gpio_put(axes[active_axis].en_pin, 1);
            move_commanded=0;
            printf("done\n");
          }
          gpio_put(STEP_pin, !gpio_get(STEP_pin));
          sleep_ms(STEP_LENGTH_ms);
          if (destination >= axes[active_axis].position)
            axes[active_axis].position++;
          else
            axes[active_axis].position--;
        }

        if(homing_commanded){
          if (gpio_get(axes[active_axis].home_pin)){
            axes[active_axis].position=0;
            destination=calculate_steps_from_switch_pos(1);
            update_dir();
            move_commanded = 1;
            homing_commanded = 0;
            continue;
          }
          else
            gpio_put(STEP_pin, !gpio_get(STEP_pin));
            sleep_ms(STEP_LENGTH_ms);
        }
    }
}


        
        