/***************************************************
 CSC C85 - UTSC RoboSoccer AI core
 
 This file contains the definition of the AI data
 structure which holds the state of your bot's AI.

 You must become familiar with this structure and
 its contents. 

 You will need to modify this file to add headers
 for any functions you added to implemet the 
 soccer playing functionality of your bot.

 Be sure to document everything you do thoroughly.

 AI scaffold: Parker-Lee-Estrada, Summer 2013
 Updated by F. Estrada, Jul. 2022

***************************************************/

#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"
#include "API/btcomm.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// Change this to match the ports your bots motors are connected to
#define LEFT_MOTOR MOTOR_D
#define RIGHT_MOTOR MOTOR_A

#define AI_SOCCER 0 	// Play soccer!
#define AI_PENALTY 1    // Go score some goals!
#define AI_CHASE 2 	    // Kick the ball around and chase it!

#define NOISE_VAR 5.0                   // Minimum amount of displacement considered NOT noise (in pixels).

#define TACTIC_ATTACK 5
#define TACTIC_DEFEND 6
#define TACTIC_SELECT 4

struct AI_data{
	// This data structure is used to hold all data relevant to the state of the AI.
	// This includes, of course, the current state, as well as the status of
	// our own bot, the opponent (if present), and the ball (if present).
	// For each agent in the game we keep a pointer to the blob that corresponds
	// to the agent (see the blob data structure in imageCapture.h), and data
	// about its old position, as well as current velocity and heading vectors.
	//
	// MIND THE NOISE.

	// Robot's playfield side id (w.r.t. the viepoint of the camera).
	int side;		// side=0 implies the robot's own side is the left side
                    // side=1 implies the robot's own side is the right side
                    // This is set based on the robot's initial position
                    // on the field
    int botCol;		// Own bot's colour. 0 - green, 1 - red

	int state;		// Current AI state

	// Object ID status for self, opponent, and ball. Just boolean 
        // values indicating whether blobs have been found for each of these
	// entities.
	int selfID;
	int oppID;
	int ballID;

	// Blob track data. Ball likely needs to be detected at each frame
	// separately. So we keep old location to estimate v
	struct blob *ball;		       // Current ball blob *NULL* if ball is not visible/found
	double old_bcx, old_bcy;	   // Previous ball cx,cy
	double bvx,bvy;			       // Ball velocity vector
	double bmx,bmy;			       // Ball motion vector
	double bdx,bdy;                // Ball heading direction (from blob shape)

	// Self track data. Done separately each frame
    struct blob *self;		       // Current self blob *NULL* if not visible/found
	double old_scx, old_scy;	   // Previous self (cx,cy)
	double svx,svy;			       // Current self [vx vy]
	double smx,smy;			       // Self motion vector
	double sdx,sdy;                // Self heading direction (from blob shape)

	// Opponent track data. Done separately each frame
    struct blob *opp;		       // Current opponent blob *NULL* if not visible/found
	double old_ocx, old_ocy;	   // Previous opponent (cx,cy)
	double ovx,ovy;			       // Current opponent [vx vy]
	double omx,omy;			       // Opponent motion vector
	double odx,ody;                // Opponent heading direction (from blob shape)
};

struct RoboAI {
	// Main AI data container. It allows us to specify which function
	// will handle the AI, and sets up a data structure to store the
	// AI's data (see above).
	void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
	void (* calibrate)(struct RoboAI *ai, struct blob *);
	struct AI_data st;
    struct displayList *DPhead;
};

/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 * 		AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 * 		cleanupAI
 */
int setupAI(int mode, int own_col, struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 * 
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

// Calibration stub
void AI_calibrate(struct RoboAI *ai, struct blob *blobs);

/* PaCode - just the function headers - see the functions for descriptions */
void id_bot(struct RoboAI *ai, struct blob *blobs);
struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col);
void track_agents(struct RoboAI *ai, struct blob *blobs);

// Display List functions
// the AI data structure provides a way for you to add graphical markers on-screen,
// the functions below add points or lines at a specified location and with the
// colour you want. Items you add will remain there until cleared. Do not mess
// with the list directly, use the functions below!
// Colours are specified as floating point values in [0,255], black is [0,0,0]
// white is [255,255,255].
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B);
struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B);
struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B);
struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B);
struct displayList *clearDP(struct displayList *head);

/****************************************************************************
 TO DO:
   Add headers for your own functions implementing the bot's soccer
   playing functionality below.
*****************************************************************************/

/* Mode enums. */
typedef enum {
  MODE_SOCCER, 
  MODE_PENALTY, 
  MODE_CHASE
} Mode;

/* State enums. */
typedef enum {
  PENALTY_TARGET_LOST, 
  PENALTY_TARGET_FOUND, 
  PENALTY_TARGET_REACHED, 
  PENALTY_GOAL_ALIGNED, 
  PENALTY_KICKED, 

  STATE_SUCCESS, 

  BALL_FLICK
} State;

/* Turn direction. */
typedef enum {
  LEFT  = -1, 
  RIGHT =  1
} TURN_DIR;

/* PID controller. */
typedef struct {
  double p; // Proportional error.
  double d; // Differential error.
  double i; // Integral error.
} PIDc;

/* Penalty. */
void penalty_target_acquire(struct RoboAI *ai);
void penalty_target_approach(struct RoboAI *ai);
void penalty_align_goal(struct RoboAI *ai);
void penalty_kick(struct RoboAI *ai);
void penalty_end(struct RoboAI *ai);

/* Soccer. */
void soccer_align_corner(struct RoboAI *ai);
bool soccer_target_acquire(struct RoboAI *ai);
void soccer_target_approach(struct RoboAI *ai);
void soccer_kick(struct RoboAI *ai);
void soccer_tactic_choose(struct RoboAI *ai);
void soccer_tactic_attack(struct RoboAI *ai);
void soccer_tactic_defend(struct RoboAI *ai);
void soccer_defend_goal(struct RoboAI *ai);
void soccer_align_goal(struct RoboAI *ai);
void soccer_align_ball(struct RoboAI *ai);
bool soccer_should_attack(struct RoboAI *ai);
void soccer_ram_goal(struct RoboAI *ai);
bool soccer_curve_around(struct RoboAI *ai);
void soccer_ball_flick(struct RoboAI *ai);

/* Mode-independant. */
bool target_approach(struct RoboAI *ai, Mode mode, double target_threshold);
bool ball_in_motion(struct RoboAI *ai);
bool stuck_backoff(struct RoboAI *ai);

/* State. */
void state_world_update(struct RoboAI *ai);
void state_error_raise(void);
void state_error_reset(void);
void state_perror_raise(const char *s);

/* Enemy. */
void enemy_threshold_reset(void);

/* PID. */
void pid_int_err_reset(void);
double pid_int_err_update(double curr_err);
double pid_u(PIDc *pid, double curr_err, double diff_err, double int_err);

/* Vector utils. */
double norm(double x, double y);
double signed_angle(double x0, double y0, double x1, double y1);

#endif
