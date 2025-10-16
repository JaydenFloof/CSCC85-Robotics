/* EV3 API
 *  Copyright (C) 2018-2019 Francisco Estrada and Lioudmila Tishkina
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#include <math.h>
#include <string.h>


int RR_turn(char direction, int speed, int degree, char motor_port_right, char motor_port_left) {
  // turn left at speed speed and degree degree, degree and speed > 0
  // direction = 'l' or 'r'

  if (direction != 'l' && direction != 'r') {
    fprintf(stderr, "turn(): direction must be 'l' or 'r'\n");
    return -1;
  }
  int turn_degree = 0;
  int rate;
  BT_read_gyro(PORT_4, 1, &turn_degree, &rate);
  while (fabs(turn_degree%360) < degree) {
    if (direction == 'r')
      BT_turn(motor_port_right, 0, motor_port_left, speed);
    else
      BT_turn(motor_port_right, speed, motor_port_left, 0);
    BT_read_gyro(PORT_4, 0, &turn_degree, &rate);
    printf("Gyro reading: angle=%d, rate=%d\n", turn_degree%360, rate);
  }
  BT_all_stop(1);
  return 0;
}

static void shift_left_add(double arr[], int len, double new_value) {
  //shift all values of arr left, drop oldest, append new_value at the end
  if (len <= 0) return;
  for (int i = 0; i < len - 1; ++i) {
    arr[i] = arr[i + 1];
  }
  arr[len - 1] = new_value;
}

int RR_straightLineMovement(int speed, int distance, char motor_port_right, char motor_port_left) {
  // move straight at speed speed and distance distance, distance > 0
  // speed > 0 for forward movement, speed < 0 for backward movement

  int angle = 0;
  int rate;
  double kp = 1;
  double ki = 0.01;
  double kd = 0.05;
  int t = 0;
  double integralErr = 0.0;
  double derivativeErr = 0.0;
  double errArray[distance];
  double PID;
  BT_read_gyro(PORT_4, 1, &angle, &rate);
  while (t < distance) {
    BT_read_gyro(PORT_4, 0, &angle, &rate);
    errArray[t] = angle;
    //integralErr += fabs(errArray[t]);
    if (t < 4)
      integralErr += fabs(errArray[t]);
    else
      //shift_left_add(errArray, 4, 0 - angle);
      integralErr = integralErr - fabs(errArray[t-4]) + fabs(errArray[t]);
    if (t > 0)
      derivativeErr = errArray[t-1] - errArray[t];
    else
      derivativeErr = 0.0;
    PID = kp*errArray[t] + ki*integralErr + kd*derivativeErr;
    printf("t=%d, err=%.2f, integralErr=%.2f, derivativeErr=%.2f, PID=%.2f\n", t, errArray[t], integralErr, derivativeErr, PID);
    BT_turn(motor_port_right, speed + PID, motor_port_left, speed - PID);
    t += 1;
  }  
  return 0;
}

int main(int argc, char *argv[]) {
  char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
  char reply[1024];
  int tone_data[50][3];

  // Reset tone data information
  for (int i = 0; i < 50; i++) {
    tone_data[i][0] = -1;
    tone_data[i][1] = -1;
    tone_data[i][2] = -1;
  }

  tone_data[0][0] = 262;
  tone_data[0][1] = 250;
  tone_data[0][2] = 1;
  tone_data[1][0] = 330;
  tone_data[1][1] = 250;
  tone_data[1][2] = 25;
  tone_data[2][0] = 392;
  tone_data[2][1] = 250;
  tone_data[2][2] = 50;
  tone_data[3][0] = 523;
  tone_data[3][1] = 250;
  tone_data[3][2] = 63;

  memset(&reply[0], 0, 1024);

// just uncomment your bot's hex key to compile for your bot, and comment the
// other ones out.
#ifndef HEXKEY
#define HEXKEY "00:16:53:56:4C:53"  // <--- SET UP YOUR EV3's HEX ID here
#endif

  BT_open(HEXKEY);

  // name must not contain spaces or special characters
  // max name length is 12 characters
  //BT_setEV3name("RobotRangers");

  BT_play_tone_sequence(tone_data);
  int r;
  int g;
  int b;
  int angle = 0;
  int rate = 0;
  if (argc > 1)
    BT_all_stop(1);
  else {
    //RR_turn('r', 100, 180, MOTOR_A, MOTOR_D);
    //RR_turn('l', 100, 25, MOTOR_A, MOTOR_D);
    RR_straightLineMovement(20, 500, MOTOR_A, MOTOR_D);
    BT_all_stop(1);
    //BT_motor_port_start(MOTOR_A | MOTOR_D, 100);
    //BT_read_colour_RGBraw_NXT(PORT_1, &r, &g, &b, &a);
    //fprintf(stderr, "Colour sensor reading: R=%d, G=%d, B=%d, A=%d\n", r, g, b, a);
    // while (1) {
    //   BT_turn(MOTOR_A, 20,  MOTOR_D, -20);
    //   BT_read_gyro(PORT_4, 0, &angle, &rate);
    //   printf("Gyro reading: angle=%d, rate=%d\n", angle%360, rate);
    // }
    //printf("Colour sensor reading: %d\n", c);
  }
    
  BT_close();
  fprintf(stderr, "Done!\n");
}
