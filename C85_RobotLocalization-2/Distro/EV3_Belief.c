#include "EV3_Localization.h"
#include "EV3_Belief.h"
#include "EV3_Model.h"

#define MAP_PADDING 2         // Padding for map shifting.
#define MIN_BELIEF 0.001      // Minimum belief for the robot to have on a single grid cell.
#define P_CORRECT_COLOR 0.85  // Probability of the color sensor reading the correct color. 

/* Normalizes beliefs. */
void normalize_beliefs() {
  /* Compute sum of beliefs. */
  double sum = 0.0;
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        if (beliefs[i+(j*sx)][k] < MIN_BELIEF) {  // Set unlikely locations to a small, non-zero probability.
          beliefs[i+(j*sx)][k] = MIN_BELIEF;
        }
        sum += beliefs[i+(j*sx)][k];
      }
    }
  }
  
  /* Normalize. */
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        beliefs[i+(j*sx)][k] /= sum;
      }
    }
  }

  return;
}

/* Updates beliefs, given action `action`. */
void action_phase(int action) {
  actionModel **m;
  switch (action) {
    case UP_DIR:
      m = FORWARD_DRIVE;
      break;
    case RIGHT_DIR:
      m = RIGHT_TURN;
      break;
    case DOWN_DIR:
      m = ABOUT_TURN;
      break;
    case LEFT_DIR:
      m = LEFT_TURN;
      break;
    default:
      fprintf(stderr, "Invalid action: %d\nDefaulting to forward drive...\n", action);
      m = FORWARD_DRIVE;
      break;
  }

  /* Buffer for updated beliefs. */
  double ***new_beliefs = (double ***) calloc(sy, sizeof(*new_beliefs));
  for (int j = 0; j < sy; j++){
    new_beliefs[j] = (double **) calloc(sx, sizeof(*new_beliefs[j]));
    for (int i = 0; i < sx; i++) {
      new_beliefs[j][i] = (double *) calloc(4, sizeof(*new_beliefs[j][i]));
    }
  }

  /* Create a belief map with padding. */
  double ***padded_belief_map = (double ***) calloc((sy + MAP_PADDING), sizeof(*padded_belief_map));
  for (int j = 0; j < sy + MAP_PADDING; j++) {
    padded_belief_map[j] = (double **) calloc((sx + MAP_PADDING), sizeof(*padded_belief_map[j]));
    for (int i = 0; i < sx + MAP_PADDING; i++) {
      padded_belief_map[j][i] = (double *) calloc(4, sizeof(*padded_belief_map[j][i]));
    }
  }

  /* Copy beliefs to the padded map. */
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        padded_belief_map[j+1][i+1][k] = beliefs[i+(j*sx)][k];
      }
    }
  }
  
  for (int d = 0; d < 4; d++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        /* 
          Action update.

          P(x_k | a_k-1): "The probability that the robot ends up 
                           at location x_k (at the start of step k 
                           of the localization process), given that 
                           it performed action  a_k-1 (at the previous
                           localization step, k-1)"
        */
        double p_ka = 0.0;
        for (int dir = 0; dir < 4; dir++) {
          for (int j_dir = 0; j_dir < 3; j_dir++) {
            for (int i_dir = 0; i_dir < 3; i_dir++) {
              p_ka += (*m[d])[dir][j_dir][i_dir] * padded_belief_map[j+j_dir][i+i_dir][dir];
            }
          }
        }

        new_beliefs[j][i][d] = p_ka;
      }
    }
  }
  
  /* Update beliefs. */
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        beliefs[i+(j*sx)][k] = new_beliefs[j][i][k];
      }
    }
  }
  normalize_beliefs();

  /* Free padded map. */
  for (int j = 0; j < sy + MAP_PADDING; j++) {
    for (int i = 0; i < sx + MAP_PADDING; i++) {
      free(padded_belief_map[j][i]);
    }
    free(padded_belief_map[j]);
  }
  free(padded_belief_map);

  /* Free computed beliefs. */
  for (int j = 0; j < sy; j++) {
    for (int i = 0; i < sx; i++) {
      free(new_beliefs[j][i]);
    }
    free(new_beliefs[j]);
  }
  free(new_beliefs);

  return;
}

/* 
  Updates beliefs, given the color sensor's readings 
  of an intersection's corners, `corner_readings`. 
*/
void read_phase(int corner_readings[4]) {
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < sy; j++) {
      for (int i = 0; i < sx; i++) {
        /* 
          Check if the corner readings match the grid. 

          P(z_k | x_k): "The probability to observe the 
                         current sensor measurements, zk, 
                         at each of the possible locations in the map."
        */
        double p_zk = 1.0;
        for (int d = 0; d < 4; d++) {
          p_zk *= (beliefs[i+(j*sx)][d] == corner_readings[d]) 
                ? P_CORRECT_COLOR 
                : (1 - P_CORRECT_COLOR);
        }

        beliefs[i+(j*sx)][k] *= p_zk;
      }
    }

    /* Rotate readings. */
    int up_tmp = corner_readings[UP_DIR];
    corner_readings[UP_DIR] = corner_readings[LEFT_DIR];
    corner_readings[LEFT_DIR] = corner_readings[DOWN_DIR];
    corner_readings[DOWN_DIR] = corner_readings[RIGHT_DIR];
    corner_readings[RIGHT_DIR] = up_tmp;
  }

  normalize_beliefs();
}

/*
  Updates beliefs, given action `action` and an intersection's 
  corner readings `corner_readings`.
*/
void update_beliefs(int action, int corner_readings[4]) {
  action_phase(action);
  read_phase(corner_readings);
}
