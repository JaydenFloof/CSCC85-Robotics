/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic.

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do:

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!
extern int sx;              // Get access to the image size from the imageCapture module
extern int sy;
int laggy=0;

#include <stdbool.h>

#define PID_BUFSIZE 5  // Buffer size for a PID controller.
#define STATE_AMT 300  // Amount of possible states.

/* Penalty parameters. */
#define PEN_RUN_UP_DIST 200.0  // Distance to begin approaching the ball.
#define PEN_APPROACH_STOP 40.0 // Distance to stop approaching the ball.
#define PEN_PWR -20            // Motor power when approaching target.
#define PEN_TURN_PWR 20        // Turning power when approaching target.
#define PEN_GOAL_ALIGN 0.25    // Maximum angle error allowed in goal alignment.
#define PEN_KICK_FRAMES 7      // Amount of frames the robot drives forward for the penalty kick.
#define OOB_SIZE 50.0          // Amount of pixels to consider the target position to be out-of-bounds.

/* Thresholds. */
#define TURN_THRESHOLD 0.4            // Minimum angle error to rotate in place (without driving).
#define WORLD_ALIGN_THRESHOLD 1.5     // Maximum angle error allowed in world state alignment.
#define HEADING_STABILITY_THRESHOLD 3 // Amount of frames with correct heading direction to be considered driving forward.

int TRANSITION_TABLE[STATE_AMT][20];                // Transition table.
void (*TRANSITIONS[STATE_AMT])(struct RoboAI *ai);  // Table of transition functions.

/*
  True, if an error was encountered in some state. 
  Must be reset manually after the error is resolved.
*/ 
bool STATE_ERROR = false;

/* Tracked blobs. */
double spx = 0.0; // Self position X.
double spy = 0.0; // Self position Y.
double sdx = 0.0; // Self direction X.
double sdy = 0.0; // Self direction Y.
double smx = 0.0; // Self motion X.
double smy = 0.0; // Self motion Y.
double opx = 0.0; // Opponent position X.
double opy = 0.0; // Opponent position Y.
double bpx = 0.0; // Ball position X.
double bpy = 0.0; // Ball position Y.

/* Target position. */
double target_x = 0.0;  // Target position X.
double target_y = 0.0;  // Target position Y.

double GOAL_X = -1.0; // Enemy goal position X.

/* Previous frame data. */
struct {
  double err; // Previous differential error.
  double sdx; // Previous self direction X.
  double sdy; // Previous self direction Y.
  double spx; // Previous self position X.
  double spy; // Previous self position Y.
  double bpx; // Previous ball position X.
  double bpy; // Previous ball position X.
} prev = { 0.0 };

/* Robot states. */
bool correct_heading_drive = false; // True, if the EV3 robot is driving forward in the correct heading direction.

/* Frame counters. */
int penalty_kick_frame = 0;    // Current penalty kick frame.
int heading_stable_frames = 0; // Number of frames the heading direction was correct.

/*
  Initial display list. 
  (Heard from a team this helps with some display bug.)
*/ 
struct displayList *init_DPhead;

double INT_ERR[PID_BUFSIZE];  // Integral error sliding window.

/* Sets all integral error values to zero. */
void pid_int_err_reset(void) {
  for (size_t i = 0; i < PID_BUFSIZE; i++) {
    INT_ERR[i] = 0.0;
  }

  return;
}

/*
  Shifts the sliding window of the integral error and 
  sets the latest error to `curr_err`.

  Returns the sum of the previous errors.
*/
double pid_int_err_update(double curr_err) {
  double sum = 0.0;

  for (size_t k = PID_BUFSIZE - 1; k > 0; k--) {
    INT_ERR[k] = INT_ERR[k-1];
    sum += INT_ERR[k];
  }
  INT_ERR[0] = curr_err;

  return sum;
}

/*
  Computes the PID output "u" of PID controller `pid`, 
  given the current, derivative, and integral error values, 
  `curr_err`, `diff_err`, `int_err`, respectively.
*/
double pid_u(PIDc *pid, double curr_err, double diff_err, double int_err) {
  return pid->p * curr_err + pid->d * diff_err + pid->i * int_err;
}

/**************************************************************
 * Display List Management
 * 
 * The display list head is kept as a pointer inside the A.I. 
 * data structure. Initially NULL (of course). It works like
 * any other linked list - anytime you add a graphical marker
 * it's added to the list, the imageCapture code loops over
 * the list and draws any items in there.
 * 
 * The list IS NOT CLEARED between frames (so you can display
 * things like motion paths that go over mutiple frames).
 * Your code will need to call clearDP() when you want this
 * list cleared.
 * 
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Blue bot
 //                   1 -> Red bot
 //                   2 -> Yellow ball
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 static double Mh[4]={-1,-1,-1,-1};
 static double mx0,my0,mx1,my1,mx2,my2;
 FILE *f;
 
 // Import calibration data from file - this will contain the colour values selected by
 // the user in the U.I.
 if (Mh[0]==-1)
 {
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
   fread(&Mh[0],4*sizeof(double),1,f);
   fclose(f);
   mx0=cos(Mh[0]);
   my0=sin(Mh[0]);
   mx1=cos(Mh[1]);
   my1=sin(Mh[1]);
   mx2=cos(Mh[2]);
   my2=sin(Mh[2]);
  }
 }

 if (Mh[0]==-1)
 {
     fprintf(stderr,"roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
     return NULL;
 }
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.9;                                               // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col==0) {vr_x=mx0; vr_y=my0;}                                                    
  else if (col==1) {vr_x=mx1; vr_y=my1;}
  else if (col==2) {vr_x=mx2; vr_y=my2;}

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 
 // Reset ID flags and agent blob pointers
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;
 
 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
 p=id_coloured_blob2(ai,blobs,ai->st.botCol);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob
  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
//  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent - whatever colour is not botCol
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	
  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 static double oldX,oldY;
 double frame_inc=1.0/5.0;
 double dist;
 
 track_agents(ai,blobs);		// Call the tracking function to find each agent

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);			// Start forward motion to establish heading
                                                // Will move for a few frames.
  
 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
// 	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;

  /* Penalty functions. */
  TRANSITIONS[101] = penalty_target_acquire;
  TRANSITIONS[102] = penalty_target_approach;
  TRANSITIONS[103] = penalty_align_goal;
  TRANSITIONS[104] = penalty_kick;
  TRANSITIONS[105] = penalty_end;

  TRANSITION_TABLE[101][PENALTY_TARGET_LOST]    = 101;  // If target not found, try again.
  TRANSITION_TABLE[102][PENALTY_TARGET_FOUND]   = 102;  // If target found, proceed to target.
  TRANSITION_TABLE[103][PENALTY_TARGET_REACHED] = 103;  // If target reached, align with goal.
  TRANSITION_TABLE[104][PENALTY_GOAL_ALIGNED]   = 104;  // If aligned with goal, kick ball.
  TRANSITION_TABLE[105][PENALTY_KICKED]         = 105;  // If ball kicked, halt motors and exit.

 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
   BT_all_stop(0);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

         
  }

    // Initialize BotInfo structures
    init_DPhead = ai->DPhead;
  }
  else
  {
    /****************************************************************************
     TO DO:
     You will need to replace this 'catch-all' code with actual program logic to
     implement your bot's state-based AI.

     After id_bot() has successfully completed its work, the state should be
     1 - if the bot is in SOCCER mode
     101 - if the bot is in PENALTY mode
     201 - if the bot is in CHASE mode

     Your AI code needs to handle these states and their associated state
     transitions which will determine the robot's behaviour for each mode.

     Please note that in this function you should add appropriate functions below
     to handle each state's processing, and the code here should mostly deal with
     state transitions and with calling the appropriate function based on what
     the bot is supposed to be doing.
    *****************************************************************************/
    //  fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
    ai->DPhead = init_DPhead;
    track_agents(ai,blobs);		// Currently, does nothing but endlessly track

    /* State data acquisition. */
    state_world_update(ai);

    /* If goal position unknown, early exit. */
    if (GOAL_X == -1.0) {
      fprintf(stderr, "error: goal not found");
      return;
    }

    /* Perform action according to current transition function. */
    (*TRANSITIONS[ai->st.state])(ai);

    /* Debug: Add graphical markers to landmarks of interest. */
    ai->DPhead = addCross(ai->DPhead, target_x, target_y, 30, 0.0, 0.0, 255.0);         // Target position for penalties.
    ai->DPhead = addCross(ai->DPhead, GOAL_X, sy/2, 30, 0.0, 255.0, 0.0);               // Center of enemy goal.
    ai->DPhead = addVector(ai->DPhead, spx, spy, sdx, sdy, 300, 255.0, 0.0, 0.0);       // Robot's facing direction.
    ai->DPhead = addCross(ai->DPhead, OOB_SIZE, OOB_SIZE, OOB_SIZE, 255.0, 0.0, 255.0); // OoB marker.
    
    /* Set previous frame data. */
    prev.spx = spx;
    prev.spy = spy;
    prev.sdx = sdx;
    prev.sdy = sdy;
    prev.bpx = bpx;
    prev.bpy = bpy;
  }
}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

/* 
  Acquires the target x,y position to be reached by the EV3 robot 
  during a penalty kick.

  The target position must necessarily be at a sufficient distance 
  from the ball, so as to not unintentionally move the ball.
*/
void penalty_target_acquire(struct RoboAI *ai) {
  state_error_reset();

  /* Compute vector from goal center to ball. */
  double vx = bpx - GOAL_X;
  double vy = bpy - (sy / 2);
  double dist = norm(vx, vy);

  /* Update target position. */
  target_x = bpx + (PEN_RUN_UP_DIST) * (vx / dist);
  target_y = bpy + (PEN_RUN_UP_DIST) * (vy / dist);

  /* Check that the target position is not out-of-bounds. */
  STATE_ERROR = ( 
    target_x <= OOB_SIZE || target_x >= (sx - OOB_SIZE) || 
    target_y <= OOB_SIZE || target_y >= (sy - OOB_SIZE)
  );

  /* Update state. */
  if (!STATE_ERROR) {
    fprintf(stderr, "Penalty target acquired: Approaching target...\n");
    ai->st.state = TRANSITION_TABLE[ai->st.state][PENALTY_TARGET_FOUND];
  } else {
    fprintf(stderr, "error: penalty target not found\n");
    ai->st.state = TRANSITION_TABLE[ai->st.state][PENALTY_TARGET_LOST];
  }

  return;
}

/*
  Approaches the target position during a penalty.
*/
void penalty_target_approach(struct RoboAI *ai) {
  state_error_reset();

  // TODO: Tune PID.
  PIDc pid = {
    .p = 1, 
    .d = 0.05, 
    .i = 0.01
  };

  if (norm(target_x - spx, target_y - spy) <= PEN_APPROACH_STOP) {  // Target reached.
    BT_all_stop(1);

    fprintf(stderr, "Target reached: Aligning with goal...\n");
    ai->st.state = TRANSITION_TABLE[ai->st.state][PENALTY_TARGET_REACHED];
  } else {  // Approach target.
    /* Compute vector from robot to target. */
    double vx = target_x - spx;
    double vy = target_y - spy;

    ai->DPhead = addVector(ai->DPhead, spx, spy, vx, vy, 200, 0.0, 255.0, 0.0);                                       // Robot to target position.
    ai->DPhead = addVector(ai->DPhead, spx, spy, GOAL_X - spx, (sy / 2) - spy, 1000, 255.0, 255.0, 0.0);  // Robot to goal center.

    double curr_err = signed_angle(sdx, sdy, vx, vy);
    if ((PI/2) - abs(curr_err) < TURN_THRESHOLD) { // Way off the correct angle: Rotate in place.
      fprintf(stderr, "Ignoring specialized PID turn: Rotating without driving...\n");

      int turn_dir = (curr_err < 0) ? RIGHT : LEFT;
      BT_turn(MOTOR_D, turn_dir * PEN_TURN_PWR, MOTOR_A, turn_dir * -(PEN_TURN_PWR));
    } else {  // Angle is close enough: Rotate while driving.
      double diff_err = curr_err - prev.err;
      double int_err = pid_int_err_update(curr_err);
      prev.err = curr_err;

      double u = pid_u(&pid, curr_err, diff_err, int_err);  // -PI <= u <= PI
      
      /* -100.0 <= left_pwr, right_pwr <= 100.0 */
      double left_pwr  = fmin(100.0, fmax(-100.0, PEN_PWR - u));
      double right_pwr = fmin(100.0, fmax(-100.0, PEN_PWR + u));
      BT_turn(MOTOR_D, left_pwr, MOTOR_A, right_pwr);
    }
  }

  return;
}

/*
  Align the EV3 robot with the goal during a penalty.
*/
void penalty_align_goal(struct RoboAI *ai) {
  state_error_reset();

  double angle = signed_angle(sdx, sdy, GOAL_X - spx, (sy / 2) - spy);
  if (abs(angle) <= PEN_GOAL_ALIGN) {  // Goal-aligned.
    BT_all_stop(1);
    
    fprintf(stderr, "Aligned to goal: Kicking ball...\n");
    ai->st.state = TRANSITION_TABLE[ai->st.state][PENALTY_GOAL_ALIGNED];
  } else {  // Turn in place.
    int turn_dir = (angle < 0) ? RIGHT : LEFT;
    BT_turn(MOTOR_D, turn_dir * PEN_TURN_PWR, MOTOR_A, turn_dir * -PEN_TURN_PWR);
  }

  return;
}

/*
  Perform a penalty kick.
*/
void penalty_kick(struct RoboAI *ai) {
  state_error_reset();

  fprintf(stderr, "Penalty kick frame: %d\n", penalty_kick_frame);
  if (penalty_kick_frame > PEN_KICK_FRAMES) { // Kick complete.
    penalty_kick_frame = 0;
    BT_all_stop(1);
    
    fprintf(stderr, "Kick complete: Halting motors and exiting...\n");
    ai->st.state = TRANSITION_TABLE[ai->st.state][PENALTY_KICKED];
  } else {  // Accelerate into the ball proportional to the penalty kick frame.
    /* Base speed + (10 * penalty_frames_elapsed) */
    BT_turn(MOTOR_D, -40 - (10 * penalty_kick_frame), MOTOR_A, -40 - (10 * penalty_kick_frame));
    penalty_kick_frame++;
  }

  return;
}

/*
  Halts all motors and exits the roboSoccer program.
*/
void penalty_end(struct RoboAI *ai) {
  BT_all_stop(1);
  exit(1);
}

/* Updates the state variables according to world events. */
void state_world_update(struct RoboAI *ai) {
  state_error_reset();

  /* Sanity check. */
  if (ai == NULL) {
    state_perror_raise("`ai` struct is null");
  }
  
  int game_mode = ai->st.state / 100;

  /* Set goal position, if uninitialized. */
  if (GOAL_X == -1.0) {
    /* 
      This is a hack. 

      If the ally goal is on the left side, then 
      the opponent goal is assumed to be *all the way* 
      on the right side of the video image, and vice versa.
    */
    GOAL_X = (ai->st.side) ? 0 : sx;
    fprintf(stderr, "Opponent goal registered as: %d\n", (int)GOAL_X);

    pid_int_err_reset();
  }

  /* Set ally robot data. */
  if (ai->st.selfID && ai->st.self != NULL) {
    /* Set position and heading direction. */
    spx = ai->st.self->cx;
    spy = ai->st.self->cy;
    smx = ai->st.self->mx;
    smy = ai->st.self->my;

    /* Set direction vector. */
    if (!sdx && !sdy) { // Unset direction vector (1st frame only).
      /* Check if facing direction aligns with the center of the field. */
      double flip = (abs(signed_angle(sx/2 - spx, sy/2 - spy, ai->st.self->dx, ai->st.self->dy)) < WORLD_ALIGN_THRESHOLD)
                  ?  1.0
                  : -1.0;
      sdx = ai->st.self->dx * flip;
      sdy = ai->st.self->dy * flip;

      fprintf(stderr, "Direction vector at 1st frame: (%f, %f)\n", sdx, sdy);
    } else {  // Previous direction vector value exists.
      /* If driving forward, set direction vector according to heading direction. */
      if (correct_heading_drive && abs(norm(spx - prev.spx, spy - prev.spy)) > 10) {
        /* Check if facing direction aligns with the heading direction. */
        double flip = (abs(signed_angle(ai->st.self->mx, ai->st.self->my, ai->st.self->dx, ai->st.self->dy)) < WORLD_ALIGN_THRESHOLD)
                    ?  1.0
                    : -1.0;
        sdx = ai->st.self->dx * flip;
        sdy = ai->st.self->dy * flip;
        
        heading_stable_frames = 0;  // Reset, since we know we're driving forward.

        fprintf(stderr, "Direction vector set according to *heading direction*\n");
      } else {  // If not driving forward, set direction vector according to its earliest known value.
        double flip = (abs(signed_angle(prev.sdx, prev.sdy, ai->st.self->dx, ai->st.self->dy)) < WORLD_ALIGN_THRESHOLD)
                    ?  1.0
                    : -1.0;
        sdx = ai->st.self->dx * flip;
        sdy = ai->st.self->dy * flip;

        fprintf(stderr, "Direction vector set according to *past direction vector*\n");
      }

      fprintf(stderr, "Driving forward?: %s\n", (correct_heading_drive) ? "true" : "false");
      fprintf(stderr, "Current direction vector: (%f, %f)\n", sdx, sdy);
      
      correct_heading_drive = false;  // Reset to prepare for next frame.
    }
  } else {
    state_perror_raise("couldn't find ally robot");
  }

  /* Set ball location. */
  if (ai->st.ballID && ai->st.ball != NULL) {
    bpx = ai->st.ball->cx;
    bpy = ai->st.ball->cy;
  } else {
    state_perror_raise("couldn't find ball");
  }

  /* Set opponent location (if appropriate for the game mode). */
  if (game_mode == MODE_SOCCER) {
    if (ai->st.oppID && ai->st.opp != NULL) {
      opx = ai->st.opp->cx;
      opy = ai->st.opp->cy;
    } else {
      state_perror_raise("couldn't find opponent robot in soccer mode");
    }
  } else {
    if (ai->st.oppID && ai->st.opp != NULL) {
      state_perror_raise("opponent robot detected in solo game mode");
    }
  }

  return;
}

/*
  Sets the state error flag to true.
*/
void state_error_raise(void) { STATE_ERROR = true; }

/*
  Sets the state error flag to false.
*/
void state_error_reset(void) { STATE_ERROR = false; }

/*
  Sets the state error flag to true 
  and prints an error message `s`.
*/
void state_perror_raise(const char *s) {
  fprintf(stderr, "error: %s\n", s);
  state_error_raise();

  return;
}

/*
  Compute the norm of `x` and `y`.
*/
double norm(double x, double y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

/*
  Returns the signed angle between (`x0`,`y0`) and (`x1`, `y1`).

  Range: [-PI, PI]
*/
double signed_angle(double x0, double y0, double x1, double y1) {
  return atan2(x0*y1 - x1*y0, x0*x1 + y0*y1);
}
