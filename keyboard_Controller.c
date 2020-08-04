#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/keyboard.h>

#define TIME_STEP 64


int main(int argc, char **argv) {
  wb_robot_init();
  int i;
  wb_keyboard_enable(TIME_STEP);
  // void wb_keyboard_disable();
  // int wb_keyboard_get_sampling_period();
  // int wb_keyboard_get_key();
  
  
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  double left_speed = 0.0;
  double right_speed = 0.0;
  printf("[UP] = forward\n");
  printf("[DOWN] = back\n");
  printf("[LEFT] = left\n");
  printf("[RIGHT] = right\n");
  
  
   while (wb_robot_step(TIME_STEP) != -1) {
      int key = wb_keyboard_get_key();
      if(key==WB_KEYBOARD_UP){
            left_speed = 3.0;
            right_speed = 3.0;
      }else if(key==WB_KEYBOARD_DOWN){
            left_speed = -3.0;
            right_speed = -3.0;
      }else if(key==WB_KEYBOARD_RIGHT){
            left_speed = 3.0;
            right_speed = -3.0;
      }else if(key==WB_KEYBOARD_LEFT){
            left_speed = -3.0;
            right_speed = 3.0;
      }else{
            left_speed = 0.0;
            right_speed = 0.0;
      }
         //key = wb_keyboard_get_key();
      wb_motor_set_velocity(wheels[0], left_speed);
      wb_motor_set_velocity(wheels[1], right_speed);
      wb_motor_set_velocity(wheels[2], left_speed);
      wb_motor_set_velocity(wheels[3], right_speed);
    }
    
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}