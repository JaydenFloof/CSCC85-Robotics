// This is modified from the LIDAR devices example to serve the
// purpose of robot localization. Original copyright notice 
// and license terms included below.
//
// This modified controller was produced by F. Estrada for
// CSC C85 - Fundamentals of Robotics and Automated Systems
// Last modified Aug. 2025

/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define TIME_STEP 32
#define LEFT 0
#define RIGHT 1

#define MIN_WALL_DIST .25       // Threshold for backing up from walls
#define S_NOISE .15             // Sensor noise for particle fitness
#define R_NOISE .1              // Resampling noise - create particles nearby!
#define D_NOISE .015            // Distance noise in meters (simulation space)
#define PI 3.1415926535
#define NUM_PARTICLES 5000     // NUmber of particles in the filter
#define MAX_SPHERES 300        // Maximum number of spheres to display (reduce if too laggy)
#define BD 7                   // Minimum image dist. between border and particles
//#define __DEBUG 0    // Turn on/off the debug print statements
                       // use step-by-step simulation for this!

unsigned char *read_PPM_map(int *sx, int *sy, char *name)
{
  // Read the image representing the maze so we can do the
  // particle filtering
  int x,y,d;
  char line[1024], *dd;
  unsigned char *im;
  FILE *f;
  
  fprintf(stderr,"Reading map image from %s\n",name);
  
  f=fopen(name,"rb+");    // Prevent problems on Windows!
  if (f<=0)
  {
    fprintf(stderr,"UNABLE to open map file!\n");
    *(sx)=0;
    *(sy)=0;
    return NULL;
  }

  dd=fgets(&line[0],1024,f);
  if (strcmp(line,"P6\n")!=0)
  {
    fprintf(stderr,"The input image is not a PPM image\n");
    *(sx)=0;
    *(sy)=0;
    return NULL;
  }

  line[0]='#';  
  while(line[0]=='#')
   dd=fgets(&line[0],1024,f);
   
  sscanf(&line[0],"%d %d",&x,&y);
  if (x<0||y<0||x>4096||y>4096)
  {
    fprintf(stderr,"MAP Image size looks wrong!\n");
    *(sx)=0;
    *(sy)=0;
    return NULL;
  }
  dd=fgets(&line[0],1024,f);
  
  im=(unsigned char *)calloc(x*y*3,sizeof(unsigned char));
  d=fread(im,x*y*3,sizeof(unsigned char),f);

  *(sx)=x;
  *(sy)=y;
  return im;    
}

void particle_resample(float *part, float *bel, int n)
{
  // Resample to create a new set of particles
  //
  // - particles: array of input particles ** THIS MUST BE UPDATED
  // - bel - array with particle beliegs ** DOES NOT need to be updated
  //                                        since immediately after resampling
  //                                        and motion, the fitness function will
  //                                        be called again
  // - n - The number of particles
  
  // TO DO: Implement this function to create a new set of particles
  // to replace the old one. The sampling is done as per the particle filter
  // rules: A particle is chosen with probability proportional to its
  // belief value.
  //
  // You may want to change the number of particles to a smaller value (say
  // 500 or so) while you're developing code, if your code is not carefully
  // written, it will take a while to update. So, think through how you want to
  // implement this function
  
  // The end result of the call to this function is that the 'part' array will
  // now contain the new set of particles. Since you're sampling from 'part'
  // you'll need to create a temporary array - be sure to free it! don't leave
  // memory leaks because this function will be called lots of times, it will
  // eat up your memory if you're not careful.
  
  return;  
}

float particle_fitness(float *parts, int i, float *gt, const float *lid, int gt_samples, int lid_samples, float noise)
{
  // Evaluate the fitness for the particle
  // This is a measure of how well the particle's GT agrees with the lidar
  // measurements, accounting for noise.
  //
  // parts - pointer to the particles array (so the orientation can be updated)
  // i - index to particle being evaluated
  // gt - pointer to the GT array, it contains 'gt_samples' entries
  //      these are in the CLOCKWISE direction starting at 12 (UP)
  //      and sampled at equally spaced intervals around a FULL circle
  // lid - pointer to the lidar image - this is a single row with 
  //       entries corresponding to the distances measured from
  //       left to right as the LIDAR is scanning in front of the
  //       robot. It contains 'lid_samples' entries
  //       NOTE - the lidar sees only 180 degrees, the ground truth
  //              contains samples for a full 360 degrees because we
  //              don't know which way the bot is facing
  // gt_samples - number of samples in the GT array
  // lid_samples - number of samples in the lidar 'image'
  // noise - sigma of the noise distribution
  //
  // Assumptions:
  //  - The orientation of the particle is not known, so this function
  //    evaluates all possible alignments between lidar data and GT
  //    and uses the orientation that yields the best agreement
  //  - Once the best-agreeing direction is found, the particle's orientation
  //    must be updated (so the action step can move it in the correct direction)
  //  - Noise is Gaussian, zero mean, with the specified noise sigma
  //
  //  IMPORTANT - the LIDAR returns 'inf' when it doesn't see any walls
  //              within its range of operation. Your code MUST check
  //              for this and do something reasonable, otherwise your
  //              particle fitness will be wrong!
  //
  //  Another important note: You'll be working with small numbers here,
  //    so if you're not careful, your beliefs can easily go to zero.
  //    So, be careful how you're managing the floating point numbers
  //    you're working with!
  //
  // HINT - the log of a number between (0 and 1) is a negative number
  //        whose magnitude gets larger as the number gets smaller.

  // TO DO: Implement this function to compute and return the belief
  //        value (fitness value) for a particle. 
  
 return 1.0;         // Return the likelihood
  
}

//// YOU DO NOT NEED TO MODIFY ANY CODE BELOW THIS POINT - but you are encouraged
//// to study it and learn how the simulation is set up, how the graphical objects
//// are handled and its properties updated, how the particle filter loop does its
//// work, and so on. 
////
//// I Will NOT ask you to explain any of the Webots code in a quiz or on the final
//// exam, but I CAN AND WILL ask you details about how the particle filter works,
//// and this includes any of the steps of the computation: fitness (belief) evaluation,
//// particle resampling, particle location updates, re-normalization of probabilities,
//// and so on. 
////
//// The point is, you will need to be able to show you fully understand a particle
//// filter and can implement it in a different context/setting/language if needed.

void particle_GT(float *GTarr, float x, float y, int n_samples, float fx, float fy, unsigned char *map, int sx, int sy)
{
  // Measure the ground truth distances around a particle located
  // at (x,y).
  // - GTArr - pointer to a 1D floating point array to store the GT distances
  // - (x,y) location of the particle whose GT we need
  // - n_samples - number of radial samples to take (depends on LIDAR h_samples)
  // - (fx,fy) - size of the room in the simulation
  // - map - MAP image
  // - (ix,iy) - size of the map image in pixels
  //
  // The distances are scaled to floor size
  // and the requested number of samples are stored in the GTarr
  // array in order, GTarr[0] corresponds to the UP direction in the image
  // direction, and the angle increases CLOCKWISE by 2*PI/n_samples
  // for each entry in the array
  
  // The particles themselves will not have a direction, instead, the
  // heading direction will be found as the one that makes the particle
  // best agree with the ground truth at its location
  
  int ix,iy, bx,by;
  float px,py; 
  float th,vx,vy, wx, wy;
  float scl,l;
   
  scl=sx/fx;             // Scale from image pixels to floor size
   
  ix=(x+(fx/2.0))*scl;         // Particle's image location
  iy=-((y-(fy/2.0))*scl);
      
  if (ix<0 || iy<0 || ix>=sx || iy>=sy)
  {
    fprintf(stderr,"Particle at (%f, %f) is out of bounds!\n",x,y);
    for (int i=0; i<n_samples; i++)
     *(GTarr+i)=(sx/2.5)/scl;
    return;
  }

  for (int i=0; i<n_samples; i++)
  {
    th=i*2.0*PI/n_samples;
    vx=0;
    vy=-1;       // Initial vector is in the upwards image direction
                 // negative because image Y increases downward!
  
   *(GTarr+i)=(sx/2.5)/scl;        // Default max distance if no walls found
                 
    // Apply a 2D rotation by theta
    wx=(cos(th)*vx)-(sin(th)*vy);
    wy=(sin(th)*vx)+(cos(th)*vy);

    for (l=0; l<sx/2.5; l+=.5)
    {
      px=ix+(l*wx);
      py=iy+(l*wy);
      bx=(int)px;
      by=(int)py;
            
      if (bx<0 || by<0 || bx>=sx || by>=sy)
      {
        *(GTarr+i)=l/scl;     
        break;
      }
      else if (*(map+((bx+(by*sx))*3)+2)!=0)
      {
        *(GTarr+i)=l/scl;
        break;
      }
    }
  }
  
}

void initParticles(float *particles, int N, double fx, double fy, int sx, int sy, unsigned char *im)
{
   // Randomly places the particles on the map
   // away from walls. 
   // - particles is a pointer to the array used to store the particles
   // - N is the number of particles to create
   // - (fx,fy) is the size of the room in the simulation
   // - (sx,sy) is the size of the map image in pixels
   // - im is a pointer to the map image
   //
   // The particles array is of size Nx3 and for each particle, it stores
   // [x, y, theta]
   //
   // To index into this array: Say you want the information for particle 'i',
   // *(particles + (3*i)) ->  X coordinate
   // *(particles + (3*i) + 1) -> Y coordinate
   // *(particles + (3*i) + 2) -> Theta
  
   double X,Y;
   int ix,iy,ok;
   
   fprintf(stderr,"Initializing %d particles, floor size is (%f, %f), image size is (%d,%d)\n",N,fx,fy,sx,sy);
   for (int i=0; i<N; i++)
   {
      ok=0;
      while (!ok)
      {
       ok=1;
       X=drand48()*fx;
       Y=drand48()*fy;
       ix=(X/fx)*sx;
       iy=(Y/fy)*sy;
       if (ix<2 || iy<2 || ix>sx-3 || iy>sy-3)   // Don't add particles at the border 
       {
         ok=0;
       }
       else
       {
         // Check we're not near a wall
         for (int ii=ix-BD; ii<=ix+BD; ii++)
         {
           for (int jj=iy-BD; jj<=iy+BD; jj++)
           {
             if (ii>=0 && jj>=0 && ii<sx && jj<sy)
             {
               if (*(im+((ii+(jj*sx))*3)+2)!=0)   // Wall assumes B is non zero
               {
                 ok=0;
               }
             }
           }
          } 
          
          if (ok)
          {
            X=X-(fx/2);
            Y=Y-(fy/2);
            *(particles+(3*i))=X;
            *(particles+(3*i)+1)=Y;
            *(particles+(3*i)+2)=0.0;
          }
       }
      }      // End while (!ok)
   }      // End for (i=...)

}

void printParticles(float *parts, int N)
{
  fprintf(stderr,"** CURRENT Particle List:\n");
  fprintf(stderr,"\n");
  for (int i=0; i<N; i++)
  {
    fprintf(stderr,"%04d - (%2.2f, %2.2f, %2.2f)\n",i,*(parts+(3*i)),*(parts+(3*i)+1),*(parts+(3*i)+2));
  }
}

void addParticleNodes(float *parts, int N, int skip, float scale, float R, float G, float B)
{
  // Add one small sphere at the correct location, with specified colour
  // and material properties - these will be used to display the particles
  // as they evolve (added once, then moved around as needed, we just need
  // the objects added to the scene at this point)
  char node_string[1024];

  WbNodeRef root_node = wb_supervisor_node_get_root();
  if (!root_node) {
      printf("Error: Could not get root node.\n");
      return;
  }

  // Get the 'children' field of the root node (which is a MultiField)
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
   if (!children_field) {
      printf("Error: Could not get children field from root node.\n");
      return;
  }
  
  for (int i=0; i<N; i+=skip)
  {
    // Set up description string for the new node
    snprintf(node_string, 1024,
           "DEF PARTICLE_%d Solid {\n"
           "  translation %f %f %f\n"
           "  children [\n"
           "    Shape {\n"
           "      appearance Appearance {\n"
           "        material Material {\n"
           "          ambientIntensity .9\n"
           "          diffuseColor %f %f %f\n"
           "          transparency %f\n"
           "        }\n"
           "      }\n"
           "      geometry Sphere {\n"
           "        radius %f\n"
           "      }\n"
           "    }\n"
           "  ]\n"
           "  name \"particle_%d\"  # Optional DEF name\n"
           "  boundingObject Sphere { radius %f }\n"
           "  physics NULL\n"
           "}",
           i,
           *(parts+(3*i)), *(parts+(3*i)+1), 1.5,
           R, G, B,
           .75,
           scale,
           i,
           scale
          );

    // Import the node string into the children field
    // The '-1' means append the new node at the end of the list of children
    wb_supervisor_field_import_mf_node_from_string(children_field, -1, node_string);
  }
  
}

int main(int argc, char **argv) {
  // iterator used to parse loops
  int i, k;


 /*************************************************************************
 *****
 *****
 *****              INITIALIZATION BLOCK
 *****
 *****
 *************************************************************************/
  
  // Values required to parse LIDAR return image
  float h_samples;                      // Horizontal samples
  float v_samples;                      // Vertical samples
  float FOV;                            // Field of View in radians
  const float *lidar_im=NULL;           // Pointer to lidar return data
  int n_skip=NUM_PARTICLES/MAX_SPHERES;  // Skip factor for displaying particles
  
  // Read map image
  unsigned char *MAP=NULL;
  int sx,sy;
  MAP=read_PPM_map(&sx, &sy, "./MazeRunner_output.ppm");
  if (MAP==NULL)
  {
    fprintf(stderr,"Unable to read map image\n");
    return 1;
  }
 
  // init Webots stuff
  wb_robot_init();

  // Try to get a handle to the floor to get the floor size
  // This needs to happen after wb_robot_init!()
  WbNodeRef FL=wb_supervisor_node_get_from_def("FLOOR");
  WbFieldRef FL_size=wb_supervisor_node_get_field(FL, "size");
  const double *fsize=wb_supervisor_field_get_sf_vec2f(FL_size);

  // Set up the LIDAR
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  h_samples=wb_lidar_get_horizontal_resolution(lidar);
  v_samples=wb_lidar_get_number_of_layers(lidar);
  FOV=wb_lidar_get_fov(lidar);

  fprintf(stderr,"Floor size (%f, %f), h_samples=%f, v_samples=%f, FOV=%f\n",*(fsize),*(fsize+1),h_samples,v_samples,FOV);

  // init distance sensors
  WbDeviceTag us[2];
  double us_values[2];
  us[LEFT] = wb_robot_get_device("us0");
  us[RIGHT] = wb_robot_get_device("us1");
  for (i = 0; i < 2; ++i)
    wb_distance_sensor_enable(us[i], TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // set empirical coefficients for collision avoidance
  double coefficients[2][2] = {{12.0, -6.0}, {-10.0, 8.0}};
  double base_speed = 25.0;

  // init speed values
  double speed[2];

  // Particle Filter SETUP
  
  // Particle information
  // The particle array is NUM_PARTICLES x 3, for (X,Y, theta)
  int gt_samples=h_samples*2*PI/FOV;
  float gt[gt_samples];
  float *particles=(float *)calloc(NUM_PARTICLES*3,sizeof(float));
  float *bel=(float *)calloc(NUM_PARTICLES,sizeof(float));
  float all_bel;
  if (particles==NULL||bel==NULL)
  {
    fprintf(stderr,"Unable to allocate particles, beliefs, or ground truth arrays\n");
    return 1;
  }
  // Initialize particle array
  initParticles(particles, NUM_PARTICLES,*(fsize),*(fsize+1),sx,sy,MAP);
  
#ifdef __DEBUG
  *(particles)=0.0;       // Put a particle at the origin (where the bot starts)
  *(particles+1)=0.0;
  printParticles(particles,NUM_PARTICLES);
#endif 

  // Add the particle objects to the simulation
  addParticleNodes(particles,NUM_PARTICLES,n_skip,.05,.5,1.0,0);
  
  fprintf(stderr,"We're live!\n");

 /*************************************************************************
 *****
 *****
 *****              SIMULATION / PARTICLE-FILTERING BLOCK
 *****
 *****
 *************************************************************************/

 int ii;
 float o_x,o_y,tt;
 float dnx,dny,len,vx,vy;
 double new_pos[3];
 double mxmi[2];
 char p_name[100];
 const double *pos;
 WbNodeRef one_p,tmp;             // Reference to one particle's sphere
 WbFieldRef p_pos;                // Reference to a particle's position

 // Get the initial position of the robot 
 WbNodeRef rob=wb_supervisor_node_get_from_def("ROBOT");
 WbFieldRef r_pos=wb_supervisor_node_get_field(rob, "translation");
 pos=wb_supervisor_field_get_sf_vec3f(r_pos); 
 fprintf(stderr,"Initial robot position at (%f, %f, %f)\n",*(pos),*(pos+1),*(pos+2));
 o_x=*(pos);
 o_y=*(pos+1);
 
  // Main loop controlling the robot, reading sensors, and updating the
  // particle filter
  while (wb_robot_step(TIME_STEP) != -1) {

    // Read distance sensors
    for (i = 0; i < 2; ++i)
      us_values[i] = wb_distance_sensor_get_value(us[i]);

    // Read LIDAR sensor
    lidar_im=wb_lidar_get_range_image(lidar);

    // Particle belief update
#pragma omp parallel for schedule(dynamic,25) private(i,gt)    
    for (i=0; i<NUM_PARTICLES; i++)
    {
      particle_GT(gt, *(particles+(3*i)), *(particles+(3*i)+1), gt_samples, *(fsize), *(fsize+1), MAP, sx,sy);
      *(bel+i)=particle_fitness(particles, i, gt, lidar_im, gt_samples, h_samples, S_NOISE);
    }

    // Normalize beliefs
    all_bel=0;
    for (i=0; i<NUM_PARTICLES; i++)
      if (*(bel+i)>all_bel) all_bel=*(bel+i);
      
    if (all_bel==0) all_bel=1.0;        // Happens if the bot gets too close to a wall... the GT gets messed up
      
    for (i=0; i<NUM_PARTICLES; i++)
    {
      *(bel+i)/=all_bel;
      *(bel+i) += 1e-6;
    }

    all_bel=0;
    mxmi[0]=-1e6;
    mxmi[1]=1e6;

    // Max and min beliefs for normalizing the colour/transparency for display
    for (i=0; i<NUM_PARTICLES; i++)
      all_bel+=*(bel+i);
    for (i=0; i<NUM_PARTICLES; i++)
    {
      *(bel+i)/=all_bel;
      if (*(bel+i)>mxmi[0]) mxmi[0]=*(bel+i);
      if (*(bel+i)<mxmi[1]) mxmi[1]=*(bel+i);
    }

    fprintf(stderr,"Belief max=%f, belief min=%f\n",mxmi[0],mxmi[1]);
    if (mxmi[0]==mxmi[1])
    {
      mxmi[0]=1.0;
      mxmi[1]=0.0;
    }
    if (mxmi[0]<-1)
    {
      fprintf(stderr,"SOMETHING WENT VERY WRONG...\n");
      for (int i=0; i<NUM_PARTICLES; i+=1)
      {
       if (isnan(*(bel+i)))
        fprintf(stderr,"%f, ",*(bel+i));
      }
      fprintf(stderr,"\n");
    }
             
    // Update particle transparency to correspond to belief probabilities
    all_bel=1.0;		// It's already normalized!

#pragma omp parallel for schedule(dynamic,25) private(i,p_name,one_p,p_pos,tmp) 
    for (i=0; i<NUM_PARTICLES; i+=n_skip)
    {
      sprintf(&p_name[0],"PARTICLE_%d",i);
      one_p=wb_supervisor_node_get_from_def(p_name);
      p_pos=wb_supervisor_node_get_field(one_p, "children");
       tmp=wb_supervisor_field_get_mf_node(p_pos,0);
         p_pos=wb_supervisor_node_get_field(tmp,"appearance");
           tmp=wb_supervisor_field_get_sf_node(p_pos);
             p_pos=wb_supervisor_node_get_field(tmp,"material");
               tmp=wb_supervisor_field_get_sf_node(p_pos);
               p_pos=wb_supervisor_node_get_field(tmp,"transparency");
                 wb_supervisor_field_set_sf_float(p_pos, 1.0-((*(bel+i)-mxmi[1])/(mxmi[0]-mxmi[1])));
               p_pos=wb_supervisor_node_get_field(tmp,"ambientIntensity");
                 wb_supervisor_field_set_sf_float(p_pos, (*(bel+i)-mxmi[1])/(mxmi[0]-mxmi[1]));     
    }

    // Motion control - idea is to avoid obstacles and steer towards 'open space'
    // This could be replaced by a path-finding/path-following block since we have
    // the graph for the maze, but not maybe right this instant
    
    // First, check if we have to back up - this happens when all our lidar measurements
    // are below some small (arbitrary) threshold
    tt=-1e6;
    ii=-1;
    for (int i=1; i<h_samples-1; i++)
      if (*(lidar_im+i-1)+*(lidar_im+i)+*(lidar_im+i+1)>tt && \
          !isinf(*(lidar_im+i)) && !isinf(*(lidar_im+i-1)) && \
          !isinf(*(lidar_im+i+1))) 
            {tt=*(lidar_im+i-1)+*(lidar_im+i)+*(lidar_im+i+1); ii=i;}

#ifdef __DEBUG
    fprintf(stderr,"Lidar readings: ");
    for (int i=0; i<h_samples; i++)
      fprintf(stderr,"%2.2f, ",*(lidar_im+i));
    fprintf(stderr,"\n");
    fprintf(stderr,"tt=%f, ii=%d\n",tt,ii);
#endif
      
    if (tt<MIN_WALL_DIST)
    {
      wb_motor_set_velocity(left_motor,-base_speed);
      wb_motor_set_velocity(right_motor,-base_speed);
    }
    else
    {     
     if (ii<(h_samples/2)-2 || ii>(h_samples/2)+1)
     {
      if (ii<(h_samples/2)) tt=-1;
      else tt=1;
      speed[LEFT]=tt*base_speed;
      speed[RIGHT]=-tt*base_speed;
      wb_motor_set_velocity(left_motor,speed[LEFT]);
      wb_motor_set_velocity(right_motor,speed[RIGHT]);       
     }
     else
     {
      tt=-((double)h_samples/2.0)+(double)ii;            // Difference between 'open space' heading and current heading

      speed[LEFT]=(base_speed*tt/(.5*h_samples));       //Adjust the constants, it's a simple P term
      speed[RIGHT]=-(base_speed*tt/(.5*h_samples));
      wb_motor_set_velocity(left_motor, base_speed + speed[LEFT]);
      wb_motor_set_velocity(right_motor, base_speed + speed[RIGHT]);    
     }
    }
    
    // Move particles and update locations - motion distance for bot
    pos=wb_supervisor_field_get_sf_vec3f(r_pos);
    dnx=(*(pos) - o_x);
    dny=(*(pos+1) - o_y);
    len=sqrt((dnx*dnx)+(dny*dny));
    
    // RESAMPLE and then move particles
    particle_resample(particles,bel,NUM_PARTICLES);

#pragma omp parallel for schedule(dynamic,25) private(i,new_pos,one_p,p_pos,p_name) 
    for (i=0; i<NUM_PARTICLES; i++)
    {
      // Get a vector in the direction of the particle as per the fitness function
      vy=sin(-(*(particles+(3*i)+2)));
      vx=cos(-(*(particles+(3*i)+2)));
      
      *(particles+(3*i)+0)+=(len+(D_NOISE*(-.5+drand48())))*vx;
      *(particles+(3*i)+1)+=(len+(D_NOISE*(-.5+drand48())))*vy;
      
      // Update the simulation!
      new_pos[0]=*(particles+(3*i)+0);
      new_pos[1]=*(particles+(3*i)+1);
      new_pos[2]=1.5;
      if (i%n_skip==0)
      {
       sprintf(&p_name[0],"PARTICLE_%d",i);
       one_p=wb_supervisor_node_get_from_def(p_name);
       p_pos=wb_supervisor_node_get_field(one_p, "translation");
       wb_supervisor_field_set_sf_vec3f(p_pos, new_pos);
      }
    }    
    o_x=*(pos);
    o_y=*(pos+1);    
  }

  free(MAP);
  free(particles);
  free(bel);
  
  wb_robot_cleanup();
  
  return 0;
}
