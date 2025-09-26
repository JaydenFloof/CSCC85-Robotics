%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSC C85 - Fundamentals of Robotics and Automated Systems
% UTSC - Fall 2021
%
% Starter code (c) F. Estrada, August 2021
% 
% Sensors and Signal Processing
%
%  You may have heard there are all kinds of plans to
% send humans to Mars. Eventually, some think we may
% establish long-term habitats on the martian surface
% occupied for long periods by 'martians'.
%
%  One of the challenges of maintaining a long term
% presence on Mars is that martian gravity is much
% weaker than Earth's, and though it's still much
% better than long-term living in space, martian
% explorers would need to keep a serious exercise
% program in order to prevent physical deterioration.
%
%  To help with this task, we've developed the
% FleetByte(tm). A device worn on a person's wrist
% that keeps track of their exercise. It's similar to
% devices you may be familiar with (or indeed which
% you may be wearing). Our goal here is to design
% the sensor and signal processing software that
% will convert the raw measurements provided by
% sensors in the device into accurate estimates
% of what the human wearing it is doing.
%
% Your task is to:
%
% a) Understand the different sensors available,
%    the values they report, and their noise profile.
% b) Implement suitable noise reduction and estimation
%    routines so as to obtain estimates of state variables
%    that are as close as possible to their actual values.
% c) Apply the ideas we discussed in lecture: Noise
%    filtering, consistency, and information redundancy
%    in order to obtain good estimates for state variables.
%
% [xyzRMS,velRMS,angRMS,hrRMS]=FleetByte(secs, map, deb)
%
%   secs - number of (virtual) seconds to run the simulation for.
%          Each call to Sim1() returns sensor readings for 1 sec,
%          so this is in effect the numbe or rounds of simulation
%          you want. 
%
%   map - Select map (1 or 2), each is a crop from the global Mars
%         elevation map from NASA - image in public domain. Note
%         that motion on the map is *not to scale*, the map corrsponds
%         to a huge area on Mars, and I want to show motion on this
%         map. So we will pretend it corresponds to an area roughly
%         .5 x .5 Km in size.
%
%  deb - If set to 1, this script will plot the returned hearrate
%          sensor output (so you can see what it looks like and think
%          about how to get a heartrate out of it), and print out
%          the sensor readings returned by Sim1(). You can add your
%          own debug/testing output as well.
%               
% - delta_t - maximum change in rover direction per unit of time, in radians
%
% Return values:
%
% xyzRMS - The RMS error for position estimates obtained over the spevified number of frames in meters
% velRMS - The RMS error for velocity estimates (Km/h)
% angRMS - The RMS error for running direction estimates (in radians)
% hrRMS - The RMS error in heartrate estimates, in BPM.
%
%  On Board Sensors:
%
%  MPS - Martian Positioning System - reports 3D position anywhere on Mars
%        to within a small displacement from actual location. Like its
%        Earthly cousin, MPS has an expected location error. For 
%        a typical wearable device, on Earth, location error is
%        within 5m of the actual location 
%        (https://www.gps.gov/systems/gps/performance/accuracy/)
%        Our FleetByte has a similar receiver, but due to the lower
%        density of Martian atmosphere, distortion due to armospheric
%        effects is lower. Under open sky this means a typical location
%        accuracy of less than 1.5m.
%
%        Note: On Mars we don't have to worry about buildings. On Earth things
%          are more difficult since buildings reflect GPS signals leading
%          to increased error in position estimates.
%
%  Heart Rate Sensor (HRS) - This one is interesting. Modern wearable 
%        HR monitors typically use light reflection from 
%        arterial blood to determine the heart rate - the
%        pulsing blood creates a periodic waveform in the 
%        reflected light. Issues with noise, low signal-to-noise
%        ratio, and effects due to skin colour, thickness, and
%        even ambient light combine to produce a fairly noisy
%        signal. The HR sensor will return an array consisting
%        of the signal measured over the last 10 seconds, from
%        which you will estimate the actual heartrate.
%        If you're very curious, this manufacturer has a
%        very thorough description of how their sensor works and
%        the different technical issues involved in computing a
%        heartrate from it ** YOU ARE NOT EXPECTED TO READ 
%        THROUGH AND IMPLEMENT THIS, IT'S THERE IN CASE YOU 
%        WANT TO LEARN MORE **
%        https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
%
%  Rate gyro (RG) - A fairly standard rate gyro, returns the measured
%              change in angle for the direction of motion (i.e.
%              tells you by how many radians this direction changed
%              in between readings). 
%
%              Somewhat noisy, but this assuming the user doesn't
%              move their arms in weird directions while running
%              it won't be affected by periodic arm motions.
%
%  ** The simulation returns to you the values measured by each
%  ** of these sensors at 1 second intervals. It's up to you to
%  ** decide how best to use/combine/denoise/filter/manipulate
%  ** the sensor readings to produce a good estimate of the actual
%  ** values of the relevant variables (which are also returned
%  ** by the simulation for the purpose of evaluating your
%  ** estimates' accuracy - needless to say, you can't use these
%  ** in any way, shape, or form, to accompish your task.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xyzRMS,velRMS,angRMS,hrRMS]=FleetByte(secs, map, deb)

pkg load image;             %%% Comment this out for MATLAB
pkg load signal;            %%% Imported for some stats functions.

close all;
%%%%%%%%%% YOU CAN ADD ANY VARIABLES YOU MAY NEED BETWEEN THIS LINE... %%%%%%%%%%%%%%%%%

persistent pos_filt;            % Filtered position. [x y z]
persistent vel_filt;            % Filtered velocity. (km/h)
persistent angle_est;           % Estimated angle. (radians)
persistent hr_filt;             % Filtered heart rate (bpm)

if isempty(pos_filt)
    pos_filt = [256 256 0.5];   % Center of the map.
    vel_filt = 10;              % Initial velocity.
    angle_est = 0;              % Initial angle.
    hr_filt = 70;               % Initial heart rate.
end

% "Smootheners". (Lower => Smoother) %
alpha_pos = 0.4;                % Smoothen position.
alpha_vel = 0.25;               % Smoothen velocity.
alpha_ang_correction = 0.15;    % Angle correction strength using displacement direction.
min_disp_for_direction = 0.5;   % Threshold to trust displacement-derived direction. (meters)

%%%%%%%%%% ... AND THIS LINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

idx=1;
while(idx<=secs)               %% Main simulation loop

 [MPS,HRS,Rg]=Sim1(map);       % Simulates 1-second of running and returns the sensor readings
                         
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % TO DO:
 %  Sim1() returns noisy readings for global position (x,y,z), a heart-rate
 %    sensor signal array for the last 10 seconds, and a value for the rate
 %    gyro (the amount of rotation in radians by which the running direction
 %    changed over the last second).
 %
 %    In the space below, write code to:
 %
 %    - Estimate as closely as possible the actual 3D position of the jogger
 %
 %    - Compute the current hear-rate (this will require some thought, make
 %      sure to look closely at the plot of HRS, and think of ways in which
 %      you can determine the heart rate from this). Remember the data 
 %      in the plot corresponds to the last 10 seconds. And, just FYI, it's
 %      based on what the actual data returned from a typical wrist-worn
 %      heart rate monitor returns. So it's fairly realistic in terms of what
 %      you'd need to process if you were actually implementing a FleetByte
 %
 %    - Estimate the running direction (huh? but the rate gyro only returns
 %      the change in angle over the last second! we don't know the initial
 %      running direction right?) - well, you don't, but you can figure it
 %      out :) - that's part of the exercise.
 %      * REFERENCE: - given a direction vector, if you want to apply a
 %         rotation by a particular angle to this vector, you simply 
 %         multiply the vector by the corresponding rotation matrix:
 %
 %            d1=R*d;
 %
 %         Where d is the input direction vector (a unit-length, column
 %         vector with 2 components). R is the rotation matrix for 
 %         the amount of rotation you want:
 %
 %           R=[cos(theta) -sin(theta)
 %              sin(theta) cos(theta)];
 %
 %         'theta' is in radians. Finally, d1 is the resulting direction vector.
 %
 %    - Estimate the running speed in Km/h - This is *not* returned by any
 %      of the sensor readings, so you have to estimate it (carefully). 
 %
 %    Goal: To get the estimates as close as possible to the real value for
 %          the relevant quantities above. The last part of the script calls
 %          the imulation code to plot the real values against your estimares
 %          so you can see how well you're doing. In particular, you want the
 %          RMS of each measurement to be as close to 0 as possible.
 %          RMS is a common measure of error, and corresponds to the square
 %          root of the average squared error between a measurement and the
 %          corresponding estimate, taken over time. 
 %    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% POSITION ESTIMATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

mps_x = MPS(1);
mps_y = MPS(2);
mps_z = MPS(3);
new_pos = [mps_x, mps_y, mps_z];

% Exponential Smoothing. %
pos_filt = (1-alpha_pos).*pos_filt + alpha_pos.*new_pos;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% VELOCITY ESTIMATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

window_size = 5;

persistent pos_hist;
persistent last_pos_for_vel;
if isempty(pos_hist)
    pos_hist = pos_filt;        
    last_pos_for_vel = pos_filt; 
end

% Keep the last `window_size` frames in position history. %
pos_hist = [pos_hist; pos_filt];      
if size(pos_hist,1) > window_size
    pos_hist = pos_hist(end-window_size+1:end,:);
end

num_frames = size(pos_hist,1);
if num_frames > 1   % Compute displacement from frames within the position window. %
    dx = pos_hist(end,1) - pos_hist(1,1);
    dy = pos_hist(end,2) - pos_hist(1,2);
    dist = sqrt(dx^2 + dy^2);
    vel_inst = (dist / (num_frames-1)) * 3.6;  % m/s -> km/h
else                % Not enough velocity data, keep initial guess. %
    vel_inst = vel_filt;
end

% Exponential smoothing. %
vel_filt = (1-alpha_vel)*vel_filt + alpha_vel*vel_inst;
vel = vel_filt;

last_pos_for_vel = pos_filt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% DIRECTION ESTIMATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

angle_est = angle_est + Rg;

persistent last_pos_for_direction;
if isempty(last_pos_for_direction)
    last_pos_for_direction = pos_filt;
end

dxh = pos_filt(1) - last_pos_for_direction(1);
dyh = pos_filt(2) - last_pos_for_direction(2);
dist_direction = sqrt(dxh^2 + dyh^2);
if dist_direction > min_disp_for_direction
    disp_angle = atan2(dyh, dxh);
    angle_diff = atan2(sin(disp_angle - angle_est), cos(disp_angle - angle_est));
    angle_est = angle_est + alpha_ang_correction * angle_diff;  % Correct towards displacement angle.
end
last_pos_for_direction = pos_filt;

angle_est = atan2(sin(angle_est), cos(angle_est));  % Normalize to [-pi, pi].
di = [cos(angle_est) sin(angle_est)];               % Unit direction vector.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HEART RATE ESTIMATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = length(HRS);
t = 0:(L-1);

smooth_HRS = sgolayfilt(HRS, 4, 101);                   % Savitzky-Golay filter.
smooth_HRS = 3 * (smooth_HRS - min(smooth_HRS) + 1);    % Shift up to avoid negative values.

[amplitude, location] = findpeaks(smooth_HRS, 'MinPeakDistance', 15);

% Ignore the first and last peaks. %
if length(amplitude) > 2
    amplitude = amplitude(2:end-1);
    location = location(2:end-1);
end

num_peaks = length(amplitude);
if num_peaks > 0
    amplitude_threshold = mean(amplitude);
else
    amplitude_threshold = 0;
end

peak_deltas = [];
location_temp = location;
for i = 1:num_peaks
    % Discard peaks outside the threshold. %
    if amplitude(i) < (amplitude_threshold - 0.5)
        location_temp(i) = -1;
    end

    % Get spacing between valid peaks. %
    if i > 1 && location_temp(i-1) ~= -1 && location_temp(i) ~= -1
        peak_deltas(end+1) = location_temp(i) - location_temp(i-1);
    end
end

persistent prev_hr_estimates;
if isempty(prev_hr_estimates)
    prev_hr_estimates = [];
end

hr = 70;

num_delta = length(peak_deltas);
if num_delta > 0
    % Weighted average of intervals. %
    weight_array = 1:num_delta;
    weighted_avg = dot(weight_array, peak_deltas) / sum(weight_array);
    hr = 7000 / weighted_avg;
    
    % Smoothen heart rate over most recent 5 frames. %
    prev_hr_estimates(end+1) = hr;
    if length(prev_hr_estimates) > 5
        if hr <= 130
            if prev_hr_estimates(end) - prev_hr_estimates(end-4) > 7
                hr = hr + 15;   % Minor correction.
            end
        end
        prev_hr_estimates = prev_hr_estimates(end-4:end);   % Keep last 5 frames.
    end
end

%%%%%%%%%%%%%%%%%%%%%%%
%%% POST-PROCESSING %%%
%%%%%%%%%%%%%%%%%%%%%%%

xyz = [
    max(0, min(pos_filt(1), 512)) % 0 <= x <= 512
    max(0, min(pos_filt(2), 512)) % 0 <= y <= 512
    max(0, pos_filt(3))           % 0 <= z < +inf
];

 if (deb==1)
     figure(5);clf;plot(HRS);
     fprintf(2,'****** For this frame: *******\n');
     fprintf(2,'MPS=[%f %f %f]\n',MPS(1),MPS(2),MPS(3));
     fprintf(2,'Rate gyro=%f\n',Rg);
    %  fprintf(2,'Estimated xyz = [%.2f %.2f %.2f]\n', xyz(1), xyz(2), xyz(3));
    %  fprintf(2,'Estimated vel = %.2f km/h\n', vel);
    %  fprintf(2,'Estimated angle = %.3f rad (%.1f deg)\n', angle_est, angle_est*180/pi);
    %  fprintf(2,'Estimated HR = %.2f bpm\n', hr);
     fprintf(2,'---> Press [ENTER] on the Matlab/Octave terminal to continue...\n');
     drawnow;
     pause;
 end;

 %%% SOLUTION:   
  
 %%%%%%%%%%%%%%%%%%  DO NOT CHANGE ANY CODE BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
 % Let's use the simulation script to plot your estimates against the real values
 % of the quantities of interest and obtain error measures - notice we ignore the
 % returned XYZ, HRSt, and Rg values since they're the same we got above.
 [t1,t2,t3,xyzRMS,velRMS,angRMS,hrRMS]=Sim1(map, xyz,hr,di,vel);
 idx=idx+1; 
end;

%%%%% Interesting links you may want to browse - I used these while designing this exercise.
% https://www.rohm.com/electronics-basics/sensor/pulse-sensor
% https://valencell.com/blog/optical-heart-rate-monitoring-what-you-need-to-know/
% https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
