/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"

/* Timesteps. */
#define VEL_TIMESTEP 0.025                // Velocity timestep.
#define ACC_TIMESTEP (VEL_TIMESTEP / 10)  // Acceleration timestep.

/* Thresholds. */
#define VAR_MAX 0.05                      // Maximum variation in a signal.
#define ROT_MAX (MAX_ROT_RATE * 180 / PI) // Maximum rotation (in radians).
#define TH_MAX  3.5                       // Maximum difference in angle.

/* Angles. */
#define TURN_TH (45.0 * PI / 180) // Angle to maintain when a side thruster is working.

/* Acceleration. */
#define MAIN_ACC (MT_ACCEL * power.MAIN)                                  // Main thruster acceleration.
#define LR_ACC_DIFF ((LT_ACCEL * power.LEFT) - (RT_ACCEL * power.RIGHT))  // Difference between left and right thruster acceleration.

/* Sampling constants. */
#define SAMPLE_AMT 100000 // Amount of sensor samples to take.
#define HIST_BUFSIZE 10   // History buffer size.

/* Simulation state variables. */
double SIM_TICKS = 0; // Elapsed simulation ticks.

/* History buffers. */
struct HistoryBuffer {
  double buf[HIST_BUFSIZE]; // Buffer.
  size_t idx;               // Index of the most recent element.
  size_t size;              // Number of entries in the buffer.
} x_hist = { {0}, 0, 0 };

/* Status flags. */
struct StatusFlags {
  bool X_OK;  // True, if the horizontal position sensor is working.
  bool Y_OK;  // True, if the vertical position sensor is working.
  bool VX_OK; // True, if the horizontal velocity sensor is working.
  bool VY_OK; // True, if the vertical velocity sensor is working.
  bool TH_OK; // True, if the angular sensor is working.
} status = { true, true, true, true, true };

/* Thruster powers. */
struct ThrusterPower {
  double MAIN;  // Main thruster power.
  double LEFT;  // Left thruster power.
  double RIGHT; // Right thruster power.
  double ROT;   // Rotation power.
} power = { 0.0, 0.0, 0.0, 0.0 };

/* Sensor data. */
struct SensorData {
  double X;   // Horizontal velocity sensor.
  double Y;   // Vertical position sensor.
  double VX;  // Horizontal velocity sensor.
  double VY;  // Vertical velocity sensor.
  double TH;  // Angular sensor.
} sensor = { 0.0, 0.0, 0.0, 0.0, 0.0 };

/* Rover phase. */
typedef enum {
  ASCENT,     // Rise to avoid obstacles.
  ALIGN_X,    // Align rover with the X-axis.
  REALIGN_X,  // Move back to align rover with the X-axis.
  DESCENT,    // Descend closer to the landing platform.
  LAND        // Turn upright and slowly glide down to land.
} Phase;

Phase phase = ASCENT; // Global rover phase.

/* Thruster identifiers. */
typedef enum {
  THR_DEF,    // All thrusters.
  THR_MAIN,   // Main thruster.
  THR_LEFT,   // Left thruster.
  THR_RIGHT   // Right thruster.
} Thruster;

/* Direction identifiers. */
typedef enum {
  DIR_DOWN,   // Down direction.
  DIR_LEFT,   // Left direction.
  DIR_RIGHT,  // Right direction.
  DIR_UP      // Up direction.
} Direction;

/* 
Updates the sensor's value and status `value` and `status`, respectively. 
Takes multiple samples of a given sensor using its corresponding sensor function `sensor_func`.
*/
bool Update_Sensor(double *value, double (*sensor_func)(void), bool *status) {
  if (*status == false) {
    return false;
  }
  
  /* Get sensor samples. */
  double sensor_samples[SAMPLE_AMT];
  double sample_sum = 0;
  double sample_max = -1;
  for (size_t i = 0; i < SAMPLE_AMT; i++) {
    sensor_samples[i] = sensor_func();
    sample_sum += sensor_samples[i];
    if (fabs(sensor_samples[i]) > sample_max) {
      sample_max = fabs(sensor_samples[i]);
    }
  }
  
  /* Compute mean and variance across sensor readings. */
  double sensor_mean = sample_sum / SAMPLE_AMT;
  double sensor_var = 0;
  for (size_t i = 0; i < SAMPLE_AMT; i++) {
    sensor_var += pow((sensor_samples[i] - sensor_mean), 2) / pow(sample_max + 2, 2);
  }
  sensor_var = sensor_var / SAMPLE_AMT;
  
  /* If the sensor varies too much, consider it faulty. */
  if (sensor_var >= VAR_MAX) {
    *status = false;
    return false;
  }

  *value = sensor_mean;
  return true;
}

/* Computes all sensor data. */
void Compute_Sensor_Data(void) {
  double y_new = sensor.Y;

  /* Predict the next sensor data. */
  struct SensorData sensor_pred = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  if (SIM_TICKS > 0) {
    /* Translational acceleration. */
    double X_Acc = MAIN_ACC * sin(sensor.TH) + LR_ACC_DIFF * cos(sensor.TH);
    double Y_Acc = (G_ACCEL - MAIN_ACC * cos(sensor.TH)) + LR_ACC_DIFF * sin(sensor.TH);

    /* Displacements. */
    double DX = sensor.VX * VEL_TIMESTEP + (X_Acc * pow(ACC_TIMESTEP, 2)) / 2;
    double DY = sensor.VY * VEL_TIMESTEP + (Y_Acc * pow(ACC_TIMESTEP, 2)) / 2;

    /* Predictions. */
    sensor_pred = {
      .X  = sensor.X + DX,
      .Y  = sensor.Y - DY,
      .VX = sensor.VX + X_Acc * ACC_TIMESTEP,
      .VY = sensor.VY - Y_Acc * ACC_TIMESTEP,
      .TH = sensor.TH + fmax(-ROT_MAX, fmin(power.ROT, ROT_MAX))
    };
  }

  /***********************/
  /* Update ALL sensors. */
  /***********************/

  if (!Update_Sensor(&sensor.X, &Position_X, &status.X_OK)) {
    if (status.VX_OK) {
      sensor.X += sensor.VX * VEL_TIMESTEP;
    } else {
      sensor.X = sensor_pred.X;
    }
  }

  if (!Update_Sensor(&sensor.Y, &Position_Y, &status.Y_OK)) {
    if (status.VY_OK) {
      sensor.Y -= sensor.VY * VEL_TIMESTEP;
    } else {
      sensor.Y = sensor_pred.Y;
    }
  }

  x_hist.buf[x_hist.idx] = sensor.X;
  x_hist.idx = (x_hist.idx + 1) % HIST_BUFSIZE;
  if (x_hist.size < HIST_BUFSIZE) {
    x_hist.size++;
  }

  if (!Update_Sensor(&sensor.VX, &Velocity_X, &status.VX_OK) && status.X_OK) {
    if (x_hist.size > 0) {
      int oldest_index = (x_hist.idx - x_hist.size + HIST_BUFSIZE) % HIST_BUFSIZE;
      double oldest_X = x_hist.buf[oldest_index];
      sensor.VX = (sensor.X - oldest_X) / (VEL_TIMESTEP * x_hist.size);
    }
  }

  if (!Update_Sensor(&sensor.VY, &Velocity_Y, &status.VY_OK)) {
    if (status.Y_OK) {
      sensor.VY =  (y_new - sensor.Y) / VEL_TIMESTEP;
    } else {
      sensor.VY = sensor_pred.VY;
    }
  }

  if (!Update_Angle()) {
    sensor.TH = sensor_pred.TH;
  }
}

/* Updates the angular sensor's value and status. */
bool Update_Angle(void) {
  if (!status.TH_OK) {
    return false;
  }

  /* Get sensor samples. */
  double angle_samples[SAMPLE_AMT];
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  double angle_max = -361.0;  // Placeholder, will get overwritten later.
  double angle_min = 361.0;   // Placeholder, will get overwritten later.
  for (size_t i = 0; i < SAMPLE_AMT; i++) {
    double angle_deg = Angle();
    angle_samples[i] = angle_deg;

    double rad = angle_deg * M_PI / 180.0;
    sin_sum += sin(rad);
    cos_sum += cos(rad);

    if (angle_deg > angle_max) {
      angle_max = angle_deg;
    }
    if (angle_deg < angle_min) {
      angle_min = angle_deg;
    }
  }

  /* If the sensor varies too much, consider it faulty. */
  if (angle_max - angle_min > TH_MAX) {
    status.TH_OK = false;
    return false;
  }

  double mean_deg = fmod((atan2(sin_sum, cos_sum) * 180.0 / M_PI) + 360.0, 360.0);
  sensor.TH = mean_deg;

  return true;
}

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/

 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y()>200) VYlim=-20;
 else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 if (Angle()>1&&Angle()<359)
 {
  if (Angle()>=180) Rotate(360-Angle());
  else Rotate(-Angle());
  return;
 }

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (Position_X()>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Left_Thruster(fabs(VXlim-Velocity_X()));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
  else
  {
   Left_Thruster(0);
   Right_Thruster(fabs(VXlim-Velocity_X()));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (Velocity_Y()<VYlim) Main_Thruster(1.0);
 else Main_Thruster(0);
}

void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/

 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=Velocity_X()*Velocity_X();
 Vmag+=Velocity_Y()*Velocity_Y();

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X()>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }

  if (Velocity_X()>0){
   Right_Thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   Left_Thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (Velocity_Y()>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
  if (Velocity_Y()>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0);
  }
 }
}
