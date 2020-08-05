/*
 * File:          starter_led.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/led.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <webots/led.h>
#include <webots/compass.h>
#include <webots/position_sensor.h>

#define TIME_STEP 64


//led info
#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
//static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};


//motors and motor sensors
static WbDeviceTag left_motor, right_motor, left_wheel_sensor, right_wheel_sensor, compass;
static double eps = 0.005;
static double totalDistance = 0;
static int x=0, z=0;
static void init_devices() {
   printf("init_devices \n");
   for (int i = 0; i < LEDS_NUMBER; i++)
     leds[i] = wb_robot_get_device(leds_names[i]);
}

double degrees(double rad){
  if(rad > 0){
    return (rad*180)/3.14159;
  }else{
    return (360 + (rad*180)/3.14159);
  }
}

double distanceMeters(double wheelDegrees){
  return ((wheelDegrees/360.0)*0.12880519);
  //0.12880519 is circumference of epuck wheel in meters
}

//Straight Movement Function
void move_forward(double distance, double speed){
  double initial = wb_position_sensor_get_value(right_wheel_sensor);
  double radius = 0.0205;
  // double rightDeg = degrees(wb_position_sensor_get_value(right_wheel_sensor));
  // double rightMeters = distanceMeters(rightDeg);
  // double leftDeg = degrees(wb_position_sensor_get_value(left_wheel_sensor));
  // double leftMeters = distanceMeters(leftDeg);
  // double avg = (rightMeters + leftMeters)/2;
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
  while(((wb_position_sensor_get_value(right_wheel_sensor)-initial)*radius) < (distance*1.02)){
    wb_robot_step(TIME_STEP);
    lightsWithDirection(0, 0, (totalDistance+((wb_position_sensor_get_value(right_wheel_sensor)-initial)*radius)), 2.05);
    printf("DISTANCE: %f\n", ((wb_position_sensor_get_value(right_wheel_sensor)-initial)*radius)+totalDistance);
  }
  totalDistance=totalDistance+(1.02*distance);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

//Turning Function
void turn_towards(double goalAngle){
  
  const double *north = wb_compass_get_values(compass);
  double currentAngle = degrees(atan2(north[0], north[2]));
  if((goalAngle-currentAngle)>0){
    //positive angle
    //360 to 0 problem SOLVED
    if((abs(goalAngle-currentAngle))<180){
    //clockwise and not cross 360/0
      wb_motor_set_velocity(left_motor, 0.3);
      wb_motor_set_velocity(right_motor, -0.3);
    }else{
    //counterclockwise and cross 360/0
      wb_motor_set_velocity(left_motor, -0.3);
      wb_motor_set_velocity(right_motor, 0.3);
    }
  }else{
    //negative angle
    if((abs(goalAngle-currentAngle))<180){
    //counterclockwise and not cross 360/0
      wb_motor_set_velocity(left_motor, -0.3);
      wb_motor_set_velocity(right_motor, 0.3);
    }else{
    //clockwise and cross 360/0
      wb_motor_set_velocity(left_motor, 0.3);
      wb_motor_set_velocity(right_motor, -0.3);
    }
  }
  while(abs(goalAngle-currentAngle)>eps){
    wb_robot_step(TIME_STEP);
    currentAngle = degrees(atan2(north[0], north[2]));
    lightsWithDirection(currentAngle, goalAngle, totalDistance, 2.05);
    printf("ANGLE: %f\n", currentAngle);
  }
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}


void lightsWithDirection(double currentAngle, double goalAngle, double currentMeters, double goalMeters){
  int led, shift;
  
  //DIRECTIONAL LEDs
  if((abs(goalAngle-currentAngle))<eps){
    shift=0;
  }else if(abs(goalAngle-currentAngle)<45+eps){
    shift=1;
  }else if(abs(goalAngle-currentAngle)<90+eps){
    shift=2;
  }else if(abs(goalAngle-currentAngle)<135+eps){
    shift=3;
  }else if(abs(goalAngle-currentAngle)<180+eps){
    shift=4;
  }else if(abs(goalAngle-currentAngle)<225+eps){
    shift=5;
  }else if(abs(goalAngle-currentAngle)<270+eps){
    shift=6;
  }else if(abs(goalAngle-currentAngle)<315+eps){
    shift=7;
  }else{
    shift=0;
  }  
  printf("SHIFT: %d\n", shift);
  
  //DISTANCE LEDs
  //detecting distance
  if((currentMeters/goalMeters)==0){
    led=-1;
  }else if(((currentMeters/goalMeters)>0) && ((currentMeters/goalMeters)<0.125)){
    led=0;
  }else if(((currentMeters/goalMeters)>0.125) && ((currentMeters/goalMeters)<0.375)){
    led=1;
  }else if(((currentMeters/goalMeters)>0.375) && ((currentMeters/goalMeters)<0.625)){
    led=2;
  }else if(((currentMeters/goalMeters)>0.625) && ((currentMeters/goalMeters)<0.875)){
    led=3;
  }else if(((currentMeters/goalMeters)>0.875) && ((currentMeters/goalMeters)>0.95)){
    led=4;
  }else{
    led=4;
    //2 front LEDs, NOT ON TOP RING
    wb_led_set(leds[8],1);
    wb_led_set(leds[9],1);
  }
  printf("LED: %d\n", led);
  
  if(led>=0){
    //LEDs ON
    for (int i = 0; i <= led; i++){
      if(i==0 || i==4){
        wb_led_set(leds[((i+shift)%8)],1);
      }else{
        wb_led_set(leds[((i+shift)%8)],1);
        wb_led_set(leds[((8-i+shift)%8)],1);
      }
    }
    
    //LEDs OFF
    for (int i = 4; i > led; i--){
      if(i==0 || i==4){
        wb_led_set(leds[((i+shift)%8)],0);
      }else{
        wb_led_set(leds[((i+shift)%8)],0);
        wb_led_set(leds[((8-i+shift)%8)],0);
      }
    }
  }
}

void vectorLocation(double angle, double magnitude){
  if((angle==90) || (angle==0)){
    magnitude=magnitude*(-1);
  }
  if((angle==90) || (angle==270)){
    z=z+magnitude;
  }else if((angle==180) || (angle==0)){
    x=x+magnitude;
  }
  printf("COORDINATE: (%d, %d)\n", z, x); 
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
 
  wb_robot_init();
  init_devices();

  
   
    // time 
    time_t s,e;
    time(&s);
    
   
   
   left_motor = wb_robot_get_device("left wheel motor");
   right_motor = wb_robot_get_device("right wheel motor");
   
   
   left_wheel_sensor = wb_robot_get_device("left wheel sensor"); 
   right_wheel_sensor = wb_robot_get_device("right wheel sensor"); 
   compass = wb_robot_get_device("compass");
   
   
   
   // enable sensors
   wb_compass_enable(compass, TIME_STEP);
   wb_position_sensor_enable(left_wheel_sensor, TIME_STEP);
   wb_position_sensor_enable(right_wheel_sensor, TIME_STEP);
   
   
  
 
   // set motors
   wb_motor_set_position(left_motor, INFINITY);
   wb_motor_set_position(right_motor, INFINITY);
  
   
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
   
  
    //Test LEDS ON
   for (int i = 0; i < LEDS_NUMBER; i++){
     wb_led_set( leds[i],1);
   }
   
   //LEDS OFF
   for (int i = 0; i < LEDS_NUMBER; i++){
     wb_led_set( leds[i],0);
   }

   
   wb_motor_set_velocity(left_motor, 0);
   wb_motor_set_velocity(right_motor, 0);
  
   
  while (wb_robot_step(TIME_STEP) != -1) {
  
   const double *north = wb_compass_get_values(compass);
   double rad = atan2(north[0], north[2]);
   double angle = degrees(rad);
   double rightDeg = degrees(wb_position_sensor_get_value(right_wheel_sensor));
   double rightMeters = distanceMeters(rightDeg);
   double leftDeg = degrees(wb_position_sensor_get_value(left_wheel_sensor));
   double leftMeters = distanceMeters(leftDeg);
   double avg = (rightMeters + leftMeters)/2;
   
   time(&e);
   
   turn_towards(180);
   move_forward(1, 4);
   vectorLocation(180, 1);
   turn_towards(90);
   move_forward(1, 4);
   vectorLocation(270, 1);
   turn_towards(300);
   turn_towards(30);
   turn_towards(270);
   
   
   
   lightsWithDirection(angle, 270, avg, 2.1288);
   
   printf(" the angle is %f\n",angle);
   printf(" left encoder %f\n",leftMeters);
   printf(" right encoder %f\n",rightMeters);
   break;
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}