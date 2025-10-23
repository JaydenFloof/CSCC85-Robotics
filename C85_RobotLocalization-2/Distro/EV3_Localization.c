/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization

 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot.

 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.

 In a nutshell, the starter code provides:

 * Reading a map from an input image (in .ppm format). The map is bordered with red,
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).

 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.

 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)

 What you must implement:

 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization

 * Basic path planning - once the robot has found its location, it must drive toward a
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---

  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours.

   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.

 What you need to understand thoroughly in order to complete this project:

 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly

 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.

 * The robot control API you will use to get your robot to move, and to acquire
   sensor data. Please see the API directory and read through the header files and
   attached documentation

 Starter code:
 F. Estrada, 2018 - for CSC C85

*/

#include "EV3_Localization.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#define NXT_COLOR_AMT 6 // Amount of colors the NXT sensor can detect.

#define CALIB_FILENAME "calibration.txt"  // Calibration filename.
#define CALIB_AMT 30                      // Amount of readings to take for a calibration.

#define TURN_SPEED 10                     // Turn speed.
#define LOC_CONFIDENCE_THRESHOLD 0.6      // Amount of confidence required to assume the robot's known location.

/* Turn direction. */
typedef enum {
  NO_TURN = -1, 
  RIGHT_TURN, 
  LEFT_TURN
} TURN_DIR;

/* Move direction. */
typedef enum {
  UP_DIR, 
  RIGHT_DIR, 
  DOWN_DIR, 
  LEFT_DIR
} MOVE_DIR;

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction

int calib_colors[NXT_COLOR_AMT][3]; // Calibrated color values.

const char *colours[NXT_COLOR_AMT] = { "BLACK", 
                                         "BLUE", 
                                         "GREEN", 
                                         "YELLOW", 
                                         "RED", 
                                         "WHITE" };

int global_angle = 0;

int RR_go_down_one_road(int speed, char motor_port_right, char motor_port_left, int maxDistance, int *angle, int targetDegree){
  int r;
  int g;
  int b;
  int a;
  char colour[8];
  //int distance = 10;
  //BT_read_colour_RGBraw_NXT(PORT_2, &r, &g, &b, &a);
  int indexColour = get_closest_color(3);
  if (indexColour == 0)
    strcpy(colour, "UNKNOWN");
  else
    strcpy(colour, colours[indexColour-1]); // -1 since colors start at 1.
  printf("Colour sensor reading: Colour=%s\n", colour);

  //int angle = 0;
  int rate;
  double kp = 1;
  double ki = 0.01;
  double kd = 0.05;
  int t = 0;
  double integralErr = 0.0;
  double derivativeErr = 0.0;
  double errArray[maxDistance];
  double PID;
  int leftPower = 0;
  int rightPower = 0;
  //BT_read_gyro(PORT_4, 1, &angle, &rate);
  while (t < maxDistance && (strcmp(colour, "BLACK") == 0 || strcmp(colour, "UNKNOWN") == 0)) {
    BT_read_gyro(PORT_4, 0, angle, &rate);
    errArray[t] = *angle - targetDegree;
    //integralErr += fabs(errArray[t]);
    if (t < 5)
      integralErr += fabs(errArray[t]);
    else
      //shift_left_add(errArray, 5, angle);
      integralErr = integralErr - fabs(errArray[t-4]) + fabs(errArray[t]);
      //integralErr = fabs(errArray[0]) + fabs(errArray[1]) + fabs(errArray[2]) + fabs(errArray[3]) + fabs(errArray[4]);
    if (t > 0)
      derivativeErr = errArray[t-1] - errArray[t];
    else
      derivativeErr = 0.0;
    PID = kp*errArray[t] + ki*integralErr + kd*derivativeErr;
    printf("t=%d, err=%.2f, integralErr=%.2f, derivativeErr=%.2f, PID=%.2f\n", t, errArray[t], integralErr, derivativeErr, PID);
    //if (PID + speed > 50) PID = 50 - speed;

    // leftPower = speed + (int)PID;
    // rightPower = speed - (int)PID;




    BT_turn(motor_port_right, speed + (int)PID, motor_port_left, speed - (int)PID);
    int indexColour = get_closest_color(3);
    if (indexColour == 0)
      strcpy(colour, "UNKNOWN");
    else
      strcpy(colour, colours[indexColour-1]); // -1 since colors start at 1.
    printf("Colour sensor reading: Colour=%s\n", colour);
    //RR_straightLineMovement(speed, distance, motor_port_right, motor_port_left);
    t += 1;
  }
  BT_all_stop(1);
  RR_return_to_intersection(speed, motor_port_right, motor_port_left, angle);
  return 0;
}

int RR_turn_down_one_road(int speed, char motor_port_right, char motor_port_left, int targetDegree, int *angle){

  //int angle = 0;
  int rate;
  double kp = 0.25
  ;
  double ki = 0.01;
  double kd = 0.05;
  //int t = 0;
  double integralErr = 0.0;
  double derivativeErr = 0.0;
  double errArray[250];
  double PID = -1000;
  int leftPower;
  int rightPower;
  int i = 0;
  const char *colour;

  colour = get_finalized_color(5);

  if (strcmp(colour, "YELLOW") != 0){
    printf("Colour sensor reading Not Yellow\n");
    return -1;
  }

  printf("\n\nCOLOUR YELLOWwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww, TURNING\n\n");


  //BT_read_gyro(PORT_4, 1, &angle, &rate);
  while (fabs(PID) > 1) {
    if (PID == -1000) {
      PID = -(*angle)/fabs(*angle);
    }
    BT_read_gyro(PORT_4, 0, angle, &rate);
    errArray[i] = *angle - targetDegree;
    //integralErr += fabs(errArray[t]);
    if (i < 5)
      integralErr += fabs(errArray[i]);
    else
      //shift_left_add(errArray, 5, angle);
      integralErr = integralErr - fabs(errArray[i-4]) + fabs(errArray[i]);
      //integralErr = fabs(errArray[0]) + fabs(errArray[1]) + fabs(errArray[2]) + fabs(errArray[3]) + fabs(errArray[4]);
    if (i > 0)
      derivativeErr = errArray[i-1] - errArray[i];
    else
      derivativeErr = 0.0;
    
    // if (fabs(speed * PID) > 50) {
    //     PID = (PID > 0) ? 50/speed : -50/speed;
    //     printf("Clamped PID: %.2f\n", PID);
    // }

    // if(speed - PID < -100) PID = 100 + speed;
    
    if (targetDegree > 0) {
      leftPower = (int)PID*speed + (((int)PID*speed)/fabs((int)PID*speed))*55 - (((int)PID*speed)/fabs((int)PID*speed))*25;           
      rightPower = -((int)PID*speed + ((PID*speed)/fabs(PID*speed))*65);
    }
    else {
      leftPower = (int)PID*speed + (((int)PID*speed)/fabs((int)PID*speed))*65;             
      rightPower = -((int)PID*speed + (((int)PID*speed)/fabs((int)PID*speed))*55 - (((int)PID*speed)/fabs((int)PID*speed))*25);
    }
    
    leftPower = leftPower*1.1;
    rightPower = rightPower*1.1;

    if (fabs(leftPower) > 100){
      leftPower = ((PID*speed)/fabs(PID*speed))*99;
    }
    if (fabs(rightPower) > 100){
      rightPower = -((PID*speed)/fabs(PID*speed))*99;
    }


    BT_turn(motor_port_right, leftPower, motor_port_left, rightPower);

    printf("Turning with speed %d and %d\n", leftPower , rightPower);
    PID = kp*errArray[i] + ki*integralErr + kd*derivativeErr;
    printf("i=%d, err=%.2f, integralErr=%.2f, derivativeErr=%.2f, PID=%.2f\n", i, errArray[i], integralErr, derivativeErr, PID);
    i += 1;
  }

  BT_all_stop(1);
  return 0;
}


//Get closest cardinal angele from current angle
int closest_cardinal_angle(int angle) {
    // Compute remainder relative to 90
    int remainder = angle % 90;

    if (angle >= 0) {
        if (remainder >= 45)
            angle += (90 - remainder);   // round up
        else
            angle -= remainder;          // round down
    } 

    else {
        if (remainder <= -45)
            angle -= (90 + remainder);   // round down (more negative)
        else
            angle -= remainder;          // round up (toward zero)
    }

    printf("\nCLOSEST CARDINAL ANGLE IS %d\n", angle);
    return angle;
}


/*
  Return the scanned color with the highest similarity 
  after scanning `n` times.
*/
int get_closest_color(int n) {
  double MSE = 100000;
  int MSE_index = -1;
  double *avg_reading = read_sanitized_color(n);
  for (int i = 0; i < NXT_COLOR_AMT; i++) {
    int SE = 0;
    for (int j = 0; j < 3; j++) {
      SE += pow(calib_colors[i][j] - avg_reading[j], 2);
    }

    // printf("get colour %d: SE = %d, MSE = %f\n", i + 1, SE, MSE);

    if (SE < MSE) {
      MSE = SE;
      MSE_index = i;
    }
    // printf("index is %d\n", MSE_index);
  }
  // free(avg_reading);

  if(MSE_index == 2){
    // printf("GREEN detected with avg R: %.2f, G: %.2f, B: %.2f\n", avg_reading[0], avg_reading[1], avg_reading[2]);
    if(avg_reading[0] > avg_reading[1]){
      MSE_index = 0; 
    }
  }

  // printf("returning\n");
  return MSE_index + 1; // +1, since color starts at 1.
}

/*
  Returns the average color reading, after scanning `n` times.
  The returned pointer must be freed by the caller.
*/
double *read_sanitized_color(int n) {
  int sum_reading[3] = { 0, 0, 0 };
  for (size_t j = 0; j < n; j++) {
    /* Get color reading. */
    int R, G, B, A;
    if (BT_read_colour_RGBraw_NXT(PORT_2, &R, &G, &B, &A) == -1) {  // Invalid color reading.
      j--;  // Try another reading.
      continue;
    }

    // printf("R: %d, G: %d, B: %d, A: %d\n", R, G, B, A);

    sum_reading[0] += R;
    sum_reading[1] += G;
    sum_reading[2] += B;
  }

  /* Compute average. */
  double *avg_reading = (double *) calloc(3, sizeof(double));
  for (size_t i = 0; i < 3; i++) {
    avg_reading[i] = sum_reading[i] / ((double) n);
  }

  printf("returning avg R: %.2f, G: %.2f, B: %.2f\n", avg_reading[0], avg_reading[1], avg_reading[2]);

  return avg_reading;
}

/* 
  Returns true, if there exists a location with belief high enough
  to reasonably conclude the robot's location, returns false otherwise.
*/
bool location_known() {
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        if (beliefs[i + (j * sx)][k] >= LOC_CONFIDENCE_THRESHOLD) {
          return true;
        }
      }
    }
  }

  return false;
}

/* 
  Go backwards back to the intersection if a RED border is hit
*/
int RR_return_to_intersection(int speed, char motor_port_right, char motor_port_left, int *current_angle){
  const char *colour = get_finalized_color(5);
  int wentDownRoad = -1;
  if(strcmp(colour, "RED") == 0){
    printf("RED DETECTED, REVERSING\n");
    // RR_adjust_angle(35, motor_port_right, motor_port_left, current_angle);
    for(int i=0; i<8; i++){
      BT_turn(MOTOR_A, -40, MOTOR_D, -40);
    }
    colour = get_finalized_color(5);
    int cardAngle = closest_cardinal_angle(*current_angle);
    while (colour != "YELLOW") {
      wentDownRoad = RR_go_down_one_road(-speed, motor_port_right, motor_port_left, 200, current_angle, cardAngle);
      RR_adjust_street(45, motor_port_right, motor_port_left, current_angle);
      colour = get_finalized_color(5);
    }
    if (wentDownRoad == 1){
      for (int i=0; i<4; i++){
        BT_turn(MOTOR_A, -50, MOTOR_D, -50);
      }
    }
    
    BT_all_stop(1);
    cardAngle = closest_cardinal_angle(*current_angle);
    RR_turn_down_one_road(1, motor_port_right, motor_port_left, cardAngle-90, current_angle);
    BT_all_stop(1);
    
    return 0;
  }
  else{
    return -1;
  }
}

/* 
  Will adjust the robot to a certain desired angle irrespective of current street (assuming the bot is already on the road)
*/
int RR_adjust_angle(int speed, char motor_port_right, char motor_port_left, int *current_angle, int custom) {
    printf("\nadjusting ONLY angle\n");
    BT_all_stop(1);
    const char* colour = get_finalized_color(3);
    int direction = 0; // -1 for left, 1 for right
    int allowance = 2;
    int rate = 0, angle = 1;

    // for(int i = 0; i < 2; i++){
    //   BT_turn(motor_port_left, speed*1.1, motor_port_right, speed*1.1);
    // }
    BT_all_stop(1);

    BT_read_gyro(PORT_4, 0 , current_angle, &rate);

    angle = custom;

    if(custom == -1){
      angle = closest_cardinal_angle(*current_angle);
    printf("current angle %d with target %d\n", *current_angle, angle);
    }
    

    if(*current_angle - angle > 0){
      // left
      direction = -1;
      while ((*current_angle - angle) > allowance){
        printf("iterating FIRST loop LEFT %d going to %d\n", *current_angle, angle);
        if((*current_angle - angle) <= allowance){
          BT_all_stop(1);
          break;
        }
        BT_turn(motor_port_left, -speed-15, motor_port_right, speed+15);
        BT_read_gyro(PORT_4, 0 , current_angle, &rate);
      }
      BT_all_stop(1);
    }

    else{
      // right
      direction = 1;
      while ((*current_angle - angle) < -allowance){
        printf("iterating FIRST right LEFT %d going to %d\n", *current_angle, angle);
        if((*current_angle - angle) >= -allowance){
          BT_all_stop(1);
          break;
        }
        BT_turn(motor_port_left, speed+15, motor_port_right, -speed-15);
        BT_read_gyro(PORT_4, 0 , current_angle, &rate);
      }
      BT_all_stop(1);
    }

    colour = get_finalized_color(3);
    BT_all_stop(1);
    BT_read_gyro(PORT_4, 0 , current_angle, &rate);

    printf("\nfinishing with cur %d and target %d\n", *current_angle, angle);
    BT_all_stop(1);
    return 0;
}


/* 
  Keeps the robot roughly aligned with the road by making small adjustments based on 
  colour and gyro readings
*/
int RR_adjust_street(int speed, char motor_port_right, char motor_port_left, int *current_angle) {
    BT_all_stop(1);
    const char* colour = get_finalized_color(3);
    int direction = 0; // -1 for left, 1 for right
    int allowance = 1;
    int rate = 0;
    int angle = 1;
    int max_iterations = 7;
    int counter = 0;

    // If we are already on the road, nothing to do
    if (strcmp("BLACK", colour) == 0 || strcmp("YELLOW", colour) == 0){
      BT_all_stop(1);
      return -1;
    }

    printf("\nADJUSTING STREEEET\n");
    angle = closest_cardinal_angle(*current_angle);

    while (strcmp("BLACK", colour) != 0 && strcmp("YELLOW", colour) != 0) {
        // for(int i = 0; i < 2; i++){
        //   BT_turn(motor_port_left, -speed*1.1, motor_port_right, -speed*1.1);
        // }
        counter = 0;

        BT_read_gyro(PORT_4, 0 , current_angle, &rate);
        //angle = closest_cardinal_angle(*current_angle);
        printf("current angles STREET %d with target %d\n", *current_angle, angle);

        if(*current_angle - angle > 0){
          // left
          direction = -1;
          colour = get_finalized_color(3);
          // while ((*current_angle - angle) > allowance){
          while(strcmp("BLACK", colour) != 0 && strcmp("YELLOW", colour )!= 0 && counter < max_iterations){
            printf("iterating FIRST loop LEFT %d going to %d\n", *current_angle, angle);
            // if((*current_angle - angle) <= allowance){
            //   BT_all_stop(1);
            //   break;
            // }
            BT_turn(motor_port_left, -speed, motor_port_right, speed);
            BT_read_gyro(PORT_4, 0 , current_angle, &rate);
            colour = get_finalized_color(3);
            counter++;
          }
          counter=0;
          BT_all_stop(1);
        }
        else{
          // right
          direction = 1;
          // while ((*current_angle - angle) < -allowance){
          while(strcmp("BLACK", colour) != 0 && strcmp("YELLOW", colour )!= 0 && counter < max_iterations){
            printf("iterating FIRST right LEFT %d going to %d\n", *current_angle, angle);
            // if((*current_angle - angle) >= -allowance){
            //   BT_all_stop(1);
            //   break;
            // }
            BT_turn(motor_port_left, speed, motor_port_right, -speed);
            BT_read_gyro(PORT_4, 0 , current_angle, &rate);
            colour = get_finalized_color(3);
            counter++;
          }
          counter=0;
          max_iterations = max_iterations + 7; // increase max iterations for next time
          BT_all_stop(1);
        }

        colour = get_finalized_color(3);
        printf("[adjust STREET] Colour after turn: %s\n", colour);
    }

    printf("\nfinishing with cur %d and target %d\n", *current_angle, angle);
    BT_all_stop(1);
    return 0;
}



int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-3 || dest_y==-3) {
  BT_open(HEXKEY);
  int angle=0, rate=0;
  BT_read_gyro(PORT_4, 1, &angle, &rate);

  while(true){
    // BT_turn(MOTOR_A, 50, MOTOR_D, 50);
    BT_read_gyro(PORT_4, 0, &angle, &rate);
    printf("angle is %d with rate as %d\n", angle, rate);
  }

 }

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 if(dest_x == -2 && dest_y == -2) {
  // Test going forward and staying on road
  printf("Testing going forward and staying on road\n");
  FILE *f = fopen(CALIB_FILENAME, "r");
  if (f == NULL) {
    perror("couldn't open" CALIB_FILENAME "\ncontinuing without it...");
  } else {
    int i = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
      int j = 0;
      char *token = strtok(line, ",");
      while (token) {
        calib_colors[i][j] = atof(token);
        token = strtok(NULL, ",");
        j++;
      }
      i++;
    }
    fclose(f);
  }

  BT_open(HEXKEY);

    // const char *colours[NXT_COLOR_AMT] = { "BLACK", 
    //                                      "BLUE", 
    //                                      "GREEN", 
    //                                      "YELLOW", 
    //                                      "RED", 
    //                                      "WHITE" };

  // 1. Black 2. Blue 3. Green 4. Yellow 5. Red 6. White
  int angle = 0;
  int targetDegree = 0;
  int targetDegreeTurn = 90;
  int rate = 0;
  int speed = 45;
  int tl, tr, bl, br;
  int i = 0;
  int add = 0;
  int turn = 0;
  int wentDownRoad = -1;

  BT_read_gyro(PORT_4, 0, &angle, &rate);

  //RR_go_down_one_road(speed, MOTOR_A, MOTOR_D, 200, &angle, targetDegree);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  while (true) {
    targetDegree = closest_cardinal_angle(angle);
    wentDownRoad = RR_go_down_one_road(speed, MOTOR_A, MOTOR_D, 200, &angle, targetDegree);

    //scan_intersection(&tl, &tr, &bl, &br, &angle);
    if (wentDownRoad == 0){
      turn = RR_turn_down_one_road(1, MOTOR_A, MOTOR_D, targetDegree + 90, &angle);
    }
    wentDownRoad = -1;
    //turn = RR_turn_down_one_road(1, MOTOR_A, MOTOR_D, targetDegree + 90, &angle);
    BT_read_gyro(PORT_4, 0, &angle, &rate);
    RR_adjust_street(45, MOTOR_A, MOTOR_D, &angle);
    //RR_adjust_angle(speed, MOTOR_A, MOTOR_D, &angle);
    
    // if (turn == 0){
    //   //targetDegree = targetDegree + 90;
    //   //targetDegreeTurn = targetDegreeTurn + 90;
    //   angle = 0;
    //   BT_read_gyro(PORT_4, 1, &angle, &rate);
    // }

    BT_read_gyro(PORT_4, 0, &angle, &rate);

    targetDegree = closest_cardinal_angle(angle);
    wentDownRoad = RR_go_down_one_road(speed, MOTOR_A, MOTOR_D, 200, &angle, targetDegree);

    //scan_intersection(&tl, &tr, &bl, &br, &angle);
    if (wentDownRoad == 0){
      turn = RR_turn_down_one_road(1, MOTOR_A, MOTOR_D, targetDegree - 90, &angle);
    }
    wentDownRoad = -1;
    BT_read_gyro(PORT_4, 0, &angle, &rate);
    RR_adjust_street(45, MOTOR_A, MOTOR_D, &angle);
    //RR_adjust_angle(speed, MOTOR_A, MOTOR_D, &angle);
    
    // if (turn == 0){
    //   //targetDegree = targetDegree + 90;
    //   //targetDegreeTurn = targetDegreeTurn + 90;
    //   angle = 0;
    //   BT_read_gyro(PORT_4, 1, &angle, &rate);
    // }
    BT_read_gyro(PORT_4, 0, &angle, &rate);

  }


  BT_all_stop(1);

  printf("\n\ngoing next\n\n");
  // scan_intersection(&tl, &tr, &bl, &br, &angle);
  // for (int i = 0; i < 10; i++){
  //   BT_turn(MOTOR_A, 40, MOTOR_D, -40);
  // } 
  // BT_all_stop(1);
  // RR_adjust_angle(50, MOTOR_A, MOTOR_D, &angle);


  // while (true) {
  //   RR_go_down_one_road(speed, MOTOR_A, MOTOR_D, 200, &angle, angle);
  //   RR_turn_down_one_road(1, MOTOR_A, MOTOR_D, angle+90, &angle);
  //   RR_adjust_angle(speed, MOTOR_A, MOTOR_D, &angle);
  // }

  

  BT_close();
  exit(1);

  // while(true){
    

  //   //RR_go_down_one_road(50, MOTOR_B, MOTOR_C, 100);
  // }
 
 }

if (dest_x==-4 && dest_y==-4) {
  BT_open(HEXKEY);
    FILE *f = fopen(CALIB_FILENAME, "r");
  if (f == NULL) {
    perror("couldn't open" CALIB_FILENAME "\ncontinuing without it...");
  } else {
    int i = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
      int j = 0;
      char *token = strtok(line, ",");
      while (token) {
        calib_colors[i][j] = atof(token);
        token = strtok(NULL, ",");
        j++;
      }
      i++;
    }
    fclose(f);
  }

  int angle = 0, rate = 0;
  BT_read_gyro(PORT_4, 0, &angle, &rate);

  find_street(&angle, 40);
  RR_go_down_one_road(40, MOTOR_A, MOTOR_D, 200, &angle, closest_cardinal_angle(angle));
  BT_all_stop(1);
  exit(1);
}

  /******************************************************************************************************************
   * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
   *   read your calibration data for use in your localization code. Skip this if you are not using calibration
   * ****************************************************************************************************************/

  FILE *f = fopen(CALIB_FILENAME, "r");
  if (f == NULL) {
    perror("couldn't open" CALIB_FILENAME "\ncontinuing without it...");
  } else {
    int i = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
      int j = 0;
      char *token = strtok(line, ",");
      while (token) {
        calib_colors[i][j] = atoi(token);
        token = strtok(NULL, ",");
        j++;
      }
      i++;
    }
    fclose(f);
  }

 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

// Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 fprintf(stderr,"All set, ready to go!\n");

/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.

  /* Placeholders. (Likely won't be used since we assume location is randomized.) */
  int x = -1;
  int y = -1;
  int dir = -1;
  robot_localization(&x, &y, &dir);
  

  // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
  BT_close();
  free(map_image);
  exit(0);
}

int find_street(int *angle, int speed)  
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */

  const char *colour = get_finalized_color(3);
  int rate = 0;

  while(strcmp(colour, "BLACK") != 0 && strcmp(colour, "YELLOW") != 0){
    BT_turn(MOTOR_A, speed, MOTOR_D, speed);
    colour = get_finalized_color(3);
  }

  printf("\nONM BLACK\n");

  RR_adjust_angle(35, MOTOR_A, MOTOR_D, angle, -1);

  printf("\nadjusted angle to gthing %d and %d\n", *angle, closest_cardinal_angle(*angle));

  for(int i=0; i<10; i++){
    BT_turn(MOTOR_A, 25, MOTOR_D, 25);
    colour = get_finalized_color(3);
    if(strcmp(colour, "BLACK") != 0){
      printf("\nread not black,breack\n");
      break;
    }
  }

  BT_all_stop(1);
  colour = get_finalized_color(3);

  if(strcmp(colour, "BLACK") != 0 && strcmp(colour, "YELLOW") != 0){
    // for(int i=0; i<5; i++){
    //   BT_turn(MOTOR_A, -35, MOTOR_D, -35);
    // }

    while(strcmp(colour, "BLACK") != 0 && strcmp(colour, "YELLOW") != 0){
      printf("getting back to black \n");
      BT_turn(MOTOR_A, -speed, MOTOR_D, speed);
      colour = get_finalized_color(3);
  }
    BT_all_stop(1);
    BT_read_gyro(PORT_4, 0, angle, &rate);
    
    printf("angle cur has %d\n", *angle);    

    RR_adjust_angle(40, MOTOR_A, MOTOR_D, angle, *angle + 45);
    printf("\n\n finsihed getting to custom angle %d\n\n", *angle);
    RR_adjust_street(40, MOTOR_A, MOTOR_D, angle);
    printf("\nFInisheda djust streeet with %d \n", *angle);
    RR_adjust_angle(40, MOTOR_A, MOTOR_D, angle, *angle + 20);
    // RR_adjust_street(40, MOTOR_A, MOTOR_D, angle);

    return 0;
  }
  return 0;
}


int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  return(0);
}

int scan_intersection(int *tl, int *tr, int *br, int *bl, int *angle)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
  int turnSpeed = 45;
  const char* colour;
  int rate;
  colour = get_finalized_color(10);

  if (strcmp(colour, "YELLOW") != 0){ //check if on intersection
    printf("Colour sensor reading Not Yellow\n");
    return -1;
  }
  int indexColour = get_closest_color(3);
  
  //Scan tl and tr
  for(int i=0; i<11; i++){
      BT_turn(MOTOR_A, -50, MOTOR_D, -50); //Move Back
  }
  BT_all_stop(1);
  printf("BACK DONW\n\n");
  
  colour = get_finalized_color(5);
  while (strcmp(colour, "BLACK") == 0 || strcmp(colour, "UNKNOWN") == 0) { // scan br
    BT_turn(MOTOR_A, -turnSpeed, MOTOR_D, turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);
  BT_read_gyro(PORT_4, 0 , angle, &rate);
  indexColour = get_closest_color(5);
  colour = get_finalized_color(5);
  *(br) = indexColour;
  printf("BR Colour index: %d %s LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n", indexColour, colours[indexColour-1]);
  
  
  while (strcmp(colour, "BLACK") != 0 && strcmp(colour, "YELLOW") != 0 && strcmp(colour, "UNKNOWN")) { // go back to road
    BT_turn(MOTOR_A, turnSpeed, MOTOR_D, -turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);
  colour = get_finalized_color(5);

  while (strcmp(colour, "BLACK") == 0 || strcmp(colour, "UNKNOWN") == 0) { // scan bl
    BT_turn(MOTOR_A, turnSpeed, MOTOR_D, -turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);
  indexColour = get_closest_color(5);
  *(bl) = indexColour;
  printf("BL Colour index: %d %s LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n", indexColour, colours[indexColour-1]);
  
  while (strcmp(colour, "BLACK") != 0 && strcmp(colour, "YELLOW") != 0 && strcmp(colour, "UNKNOWN")) { // go back to road
    BT_turn(MOTOR_A, -turnSpeed, MOTOR_D, turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);

  RR_adjust_angle(50, MOTOR_A, MOTOR_D, angle, -1);
  //RR_adjust_street(50,MOTOR_A, MOTOR_D, angle);
  RR_go_down_one_road(40, MOTOR_A, MOTOR_D, 200, angle, *angle);

  colour = get_finalized_color(5);
  while (colour == "YELLOW") {
    BT_turn(MOTOR_A, 50, MOTOR_D, 50);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);

  for(int i=0; i<3; i++){
    BT_turn(MOTOR_A, 50, MOTOR_D, 50); //Move Forward
  }
  BT_all_stop(1);

  // scan TR and TL
  while (strcmp(colour, "BLACK") == 0 || strcmp(colour, "UNKNOWN") == 0) { // scan tr
    BT_turn(MOTOR_A, -turnSpeed, MOTOR_D, turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);
  indexColour = get_closest_color(5);
  printf("TR Colggggggggggggggggggggggggggggggggggggggggggggggggggggggggour index: %d\n", indexColour);
  *(tr) = indexColour;

  while (strcmp(colour, "BLACK") != 0) { // go back to road
    BT_turn(MOTOR_A, turnSpeed, MOTOR_D, -turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);

  while (strcmp(colour, "BLACK") == 0 || strcmp(colour, "UNKNOWN") == 0) { // scan tl
    BT_turn(MOTOR_A, turnSpeed, MOTOR_D, -turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);
  indexColour = get_closest_color(5);
  *(tl) = indexColour;
  printf("TL Colour indgamkwgggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggex: %d\n", indexColour);

  while (strcmp(colour, "BLACK") != 0) { // go back to road
    BT_turn(MOTOR_A, -turnSpeed, MOTOR_D, turnSpeed);
    colour = get_finalized_color(5);
  }
  BT_all_stop(1);

  RR_adjust_angle(50, MOTOR_A, MOTOR_D, angle, -1);
  //RR_adjust_street(50,MOTOR_A, MOTOR_D, angle);
  RR_go_down_one_road(-40, MOTOR_A, MOTOR_D, 200, angle, *angle);
  
 return(0);
}

const char *get_finalized_color(int n) {
    int index = get_closest_color(n);

    if (index < 1) {
        fprintf(stderr, "Invalid color index, setting to UNKNOWN: %d\n", index);
        return "UNKNOWN";
    } else {
        const char *colour = colours[index - 1];
        printf("Closest color is: %s\n", colour);
        return colour;
    }
}


int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
  return(0);
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

    // printf("Starting robot localization...\n");

    // int localized = 0;
    // int corner_readings[4];   // top-left, top-right, bottom-right, bottom-left
    // int action = UP_DIR;      // assume initial facing direction
    // int step = 0;
    // int angle = 0, rate = 0;
    // BT_read_gyro(PORT_4, 1, &angle, &rate);

    // // === 1. Get onto a street ===
    // printf("Finding street...\n");
    // find_street(&angle, 40);  // aligns robot with a street (speed 40)

    // // === 2. Initialize beliefs uniformly ===
    // for (int k = 0; k < 4; k++) {
    //     for (int j = 0; j < sy; j++) {
    //         for (int i = 0; i < sx; i++) {
    //             beliefs[i + (j * sx)][k] = 1.0 / (sx * sy * 4);
    //         }
    //     }
    // }
    // normalize_beliefs();
    // printf("Initial uniform belief distribution created.\n");

    // // === 3. Localization loop ===
    // while (!localized)
    // {
    //     printf("\n--- Step %d ---\n", step++);

    //     // (a) Move physically to the next intersection
    //     drive_along_street();
    //     BT_all_stop(1);

    //     // (b) Sense environment
    //     printf("Scanning intersection...\n");
    //     scan_intersection(&corner_readings[0], &corner_readings[1], &corner_readings[2], &corner_readings[3], &angle);

    //     // (c) Update beliefs using Bayesian filter
    //     printf("Updating beliefs...\n");
    //     update_beliefs(action, corner_readings);

    //     // (d) Check if robot is localized
    //     double max_belief = 0.0;
    //     int best_i = -1, best_j = -1, best_dir = -1;

    //     for (int k = 0; k < 4; k++) {
    //         for (int j = 0; j < sy; j++) {
    //             for (int i = 0; i < sx; i++) {
    //                 double b = beliefs[i + (j * sx)][k];
    //                 if (b > max_belief) {
    //                     max_belief = b;
    //                     best_i = i;
    //                     best_j = j;
    //                     best_dir = k;
    //                 }
    //             }
    //         }
    //     }

    //     printf("Highest belief: %.3f at (%d, %d), dir=%d\n", max_belief, best_i, best_j, best_dir);

    //     // (e) Decide if localization confidence is high enough
    //     if (max_belief > 0.60) {   // threshold can be tuned
    //         localized = 1;
    //         *robot_x = best_i;
    //         *robot_y = best_j;
    //         *direction = best_dir;
    //         printf("Robot localized with high confidence!\n");
    //         break;
    //     }

    //     // (f) Otherwise, explore further — pick a turn direction and continue
    //     printf("Not confident yet, turning and exploring...\n");
    //     turn_at_intersection(RIGHT_TURN);   // example: always turn right for exploration
    //     action = RIGHT_DIR;
    // }

    // printf("Localization complete.\n");
    return 0;
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  return 0;
}


void calibrate_sensor(void)
{
  /*
   * This function is called when the program is started with -1  -1 for the target location.
   *
   * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
   * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
   * level.
   *
   * The principle is - Your code should allow you to sample the different colours in the map, and store representative
   * values that will help you figure out what colours the sensor is reading given the current conditions.
   *
   * Inputs - None
   * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
   *
   * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
   */

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr,"Calibration function called!\n");

  BT_open(HEXKEY);
  
  /* 
    WARNING: Old calibration file will be OVERWRITTEN. 

    Make sure you don't need the old calibration values 
    before calling this function!
  */
  
  // FILE *fp = fopen(CALIB_FILENAME, "w");
  // if (fp == NULL) {
  //   perror("couldn't open" CALIB_FILENAME "\nexiting...");
  //   exit(1);
  // }

  // const char *colours[NXT_COLOR_AMT] = { "BLACK", 
  //                                        "BLUE", 
  //                                        "GREEN", 
  //                                        "YELLOW", 
  //                                        "RED", 
  //                                        "WHITE" };
  
  // for (size_t i = 0; i < NXT_COLOR_AMT; i++) {
  //   printf("Place the NXT sensor over the color: %s, then press any key to continue.", colours[i]);
  //   getchar();

  //   double *avg_reading = (double *) calloc(3, sizeof(double));
  //   avg_reading = read_sanitized_color(CALIB_AMT);
  //   fprintf(fp, "%f,%f,%f\n", avg_reading[0], avg_reading[1], avg_reading[2]);
  //   free(avg_reading);
  // }
  printf("Colour calibration complete! Data saved to: %s\n", CALIB_FILENAME);
  printf("Place the Gyro sensor facing True North, then press any key to continue.");
  getchar();

  int rate = 0;
  BT_read_gyro(PORT_4, 1, &global_angle, &rate);


  // fclose(fp);
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.return(-1);
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}