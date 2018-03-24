clear variables

FILENAME = 'BS1';
CAL_FILENAME = 'IMU_Squat_Calibration_File';
LIFT = 'BS'; % 'BS' or 'FS'
MASS_BAR = 405;
UNITS = 'lbs';

REP = 1; % Rep to be analyzed

SUBJECT_HEIGHT = 1.8542; % meters
SUBJECT_MASS = 100.0; % kg

g = 9.81;

if strcmp(UNITS,'lbs')
    MASS_BAR = MASS_BAR * .453592;
end

ON = 1;
OFF = 0;
fs = 80; % approximate sampling frequency
fT = 10; % approximate sampling period in ms

[L_TRUNK, L_THIGH, L_SHANK, MASS_TRUNK, MASS_THIGH, MASS_SHANK] = subject_parameters(SUBJECT_HEIGHT, SUBJECT_MASS, 'male');

% Custom Segment Parameters
if (ON)
L_TRUNK = .2877 * SUBJECT_HEIGHT; 
L_THIGH = .2534 * SUBJECT_HEIGHT;
L_SHANK = .2500 * SUBJECT_HEIGHT;
end

DCOM_TRUNK = .569;
DCOM_THIGH = 1 - .428;
K_TRUNK = .406;
K_THIGH = .323;

I_TRUNK = MASS_TRUNK * K_TRUNK^2 * L_TRUNK^2;
I_THIGH = MASS_TRUNK * K_THIGH^2 * L_THIGH^2;

THIGH_DATA = csvread(strcat(FILENAME,'_1.csv'));
TRUNK_DATA = csvread(strcat(FILENAME,'_2.csv'));

THIGH_CAL = csvread(strcat(CAL_FILENAME,'_1.csv'));
TRUNK_CAL = csvread(strcat(CAL_FILENAME,'_2.csv'));

calN1 = length(THIGH_CAL);
calN2 = length(TRUNK_CAL);

calS1 = round(.25*calN1);
calS2 = round(.25*calN2);
calE1 = round(.75*calN1);
calE2 = round(.75*calN2);

[thigh_cal_t] = imu_extract_time(THIGH_CAL);
[thigh_cal_Ex, thigh_cal_Ey, thigh_cal_Ez] = imu_extract_euler(THIGH_CAL);
[trunk_cal_t] = imu_extract_time(TRUNK_CAL);
[trunk_cal_Ex, trunk_cal_Ey, trunk_cal_Ez] = imu_extract_euler(TRUNK_CAL);

thigh_cal_Ez = thigh_cal_Ez(calS1:calE1);
trunk_cal_Ez = trunk_cal_Ez(calS2:calE2);

THIGH_CAL_Ez = mean(thigh_cal_Ez);
TRUNK_CAL_Ez = mean(trunk_cal_Ez);

THIGH_OFFSET = 90 - THIGH_CAL_Ez;
TRUNK_OFFSET = 90 - TRUNK_CAL_Ez;

N_THIGH = length(THIGH_DATA);
N_TRUNK = length(TRUNK_DATA);

% drop first/last line
THIGH_DATA = THIGH_DATA(2:N_THIGH-1,:);
TRUNK_DATA = TRUNK_DATA(2:N_TRUNK-1,:);
    
% Extract relevant data from segment .csv files

[thigh_t] = imu_extract_time(THIGH_DATA);
[thigh_Ex, thigh_Ey, thigh_Ez] = imu_extract_euler(THIGH_DATA);

[trunk_t] = imu_extract_time(TRUNK_DATA);
[trunk_Ex, trunk_Ey, trunk_Ez] = imu_extract_euler(TRUNK_DATA);

N_THIGH = length(THIGH_DATA);
N_TRUNK = length(TRUNK_DATA);

trial_end = min([thigh_t(N_THIGH),trunk_t(N_TRUNK)]);

trial_t = 0:fT:trial_end;
N = length(trial_t);
FRAME = 1:N;

fc = 3;
order = 4;
[b,a] = butter(order,fc/(fs/2));

[ thigh_Ex, thigh_Ey, thigh_Ez ] = imu_interp_orientation( thigh_t, thigh_Ex, thigh_Ey, thigh_Ez, trial_t);
[ trunk_Ex, trunk_Ey, trunk_Ez ] = imu_interp_orientation( trunk_t, trunk_Ex, trunk_Ey, trunk_Ez, trial_t);

THIGH_ANG = 180 - (thigh_Ez + THIGH_OFFSET);
    
TRUNK_ANG = trunk_Ez + TRUNK_OFFSET;

dt = .01;

THIGH_W = zeros(1,N);
TRUNK_W = zeros(1,N);

for i = 2:N-1
    THIGH_W(i) = (THIGH_ANG(i+1)-THIGH_ANG(i-1))/(2*dt);
    TRUNK_W(i) = (TRUNK_ANG(i+1)-TRUNK_ANG(i-1))/(2*dt);
end

THIGH_W = filtfilt(b,a,THIGH_W);
TRUNK_W = filtfilt(b,a,TRUNK_W);

THIGH_W = THIGH_W/57.3;
TRUNK_W = TRUNK_W/57.3;

THIGH_ALPHA = zeros(1,N);
TRUNK_ALPHA = zeros(1,N);

for i = 2:N-1
    THIGH_ALPHA(i) = (THIGH_W(i+1)-THIGH_W(i-1))/(2*dt);
    TRUNK_ALPHA(i) = (TRUNK_W(i+1)-TRUNK_W(i-1))/(2*dt);
end

THIGH_ALPHA = filtfilt(b,a,THIGH_ALPHA);
TRUNK_ALPHA = filtfilt(b,a,TRUNK_ALPHA);

% Find squat reps
THRESHOLD_THIGH = THIGH_ANG > 160;
THRESHOLD_THIGH = THIGH_ANG .* THRESHOLD_THIGH;

SQUAT_COUNT = 0;
n = 0;
while n < N
    n = n + 1;
    if THRESHOLD_THIGH(n) > 0
        LS = n;
        while THRESHOLD_THIGH(n) > 0
            n = n + 1;
        end
        LE = n - 1;
        SQUAT_COUNT = SQUAT_COUNT + 1;
        SQUAT(SQUAT_COUNT,1) = SQUAT_COUNT;
        SQUAT(SQUAT_COUNT,2) = LS;
        SQUAT(SQUAT_COUNT,3) = LE;
        SQUAT(SQUAT_COUNT,4) = find(THRESHOLD_THIGH(LS:LE) == max(THRESHOLD_THIGH(LS:LE))) + (LS - 1);
        for i = LS:-1:1
            if THIGH_ANG(i) < 100
                SS = i;
                break
            end
        end
        for i = LE:1:N
            if THIGH_ANG(i) < 100
                SE = i;
                break
            end
        end
        SQUAT(SQUAT_COUNT,5) = SS;
        SQUAT(SQUAT_COUNT,6) = SE;
    end
end

max_depth = max(THIGH_ANG);
nDepthMax = find(THIGH_ANG == max_depth);

START = SQUAT(REP,5);
END = SQUAT(REP,6);
ECC_END = SQUAT(REP,4);
CON_START = ECC_END; % Placeholder until account for pause reps

S = START;
E = END;

figure(1)
plot(FRAME(S:E),THIGH_ANG(S:E),FRAME(S:E),TRUNK_ANG(S:E))
title('Segment Orientations  [Full Trial]')
legend('Thigh','Trunk')

figure(2)
plot(FRAME(S:E),THIGH_W(S:E),FRAME(S:E),TRUNK_W(S:E));
title('Segment Angular Velocities  [Full Trial]')
legend('Thigh','Trunk')

figure(3)
plot(FRAME(S:E),THIGH_ALPHA(S:E),FRAME(S:E),TRUNK_ALPHA(S:E));
title('Segment Angular Accelerations   [Full Trial]')
legend('Thigh','Trunk')

figure(4)
plot(FRAME(S:E),THRESHOLD_THIGH(S:E))
title('Thresholded Thigh')

max_depth = max(THIGH_ANG);
nDepthMax = find(THIGH_ANG == max_depth);

START = SQUAT(REP,5);
END = SQUAT(REP,6);
ECC_END = SQUAT(REP,4);
CON_START = ECC_END; % Placeholder until account for pause reps

if (ON)
    
%CON_START = nDepthMax;
S = START;
E = CON_START;

figure(4)
plot(FRAME(S:E),THIGH_ANG(S:E),FRAME(S:E),TRUNK_ANG(S:E))
title('Segment Orientations  [Eccentric Phase]')
legend('Thigh','Trunk')

figure(5)
plot(FRAME(S:E),THIGH_W(S:E),FRAME(S:E),TRUNK_W(S:E));
title('Segment Angular Velocities  [Eccentric Phase]')
legend('Thigh','Trunk')

figure(6)
plot(FRAME(S:E),THIGH_ALPHA(S:E),FRAME(S:E),TRUNK_ALPHA(S:E));
title('Segment Angular Accelerations   [Eccentric Phase]')
legend('Thigh','Trunk')

S = CON_START;
E = END;

figure(7)
plot(FRAME(S:E),THIGH_ANG(S:E),FRAME(S:E),TRUNK_ANG(S:E))
title('Segment Orientations  [Concentric Phase]')
legend('Thigh','Trunk')

figure(8)
plot(FRAME(S:E),THIGH_W(S:E),FRAME(S:E),TRUNK_W(S:E));
title('Segment Angular Velocities  [Concentric Phase]')
legend('Thigh','Trunk')

figure(9)
plot(FRAME(S:E),THIGH_ALPHA(S:E),FRAME(S:E),TRUNK_ALPHA(S:E));
title('Segment Angular Accelerations  [Concentric Phase]')
legend('Thigh','Trunk')

MaxTilt = min(TRUNK_ANG(S:E));
nTilt = find(MaxTilt == TRUNK_ANG);
MaxTilt = 90 - MaxTilt;

%%%%%%
A_BAR = 0;
F_BAR = MASS_BAR * (g + A_BAR);
F_SHO = F_BAR;
F_HIP = F_SHO + (MASS_TRUNK * g);
F_KNEE = F_HIP + (MASS_THIGH * g);

if strcmp(LIFT,'BS')
    bar_offset = -.01942 * SUBJECT_HEIGHT;
    bar2hip = cosd(TRUNK_ANG) * L_TRUNK + bar_offset;
elseif strcmp(LIFT,'FS')
    bar_offset = .04854 * SUBJECT_HEIGHT;
    bar2hip = (cosd(TRUNK_ANG) * L_TRUNK) + bar_offset;
end
knee2hip = -cosd(THIGH_ANG) * L_THIGH;
bar2knee = (knee2hip - bar2hip);


M_HIP = (F_SHO * bar2hip) + (MASS_TRUNK * g * DCOM_TRUNK * bar2hip);
M_KNEE = (F_HIP * knee2hip) + (MASS_THIGH * g * DCOM_THIGH * knee2hip) - M_HIP;
%M_KNEE = - (F_HIP * knee2hip) - (mass_thigh * g * Dcom_thigh * knee2hip) + M_HIP; 

figure(10)
plot(FRAME(S:E),bar2hip(S:E),FRAME(S:E),knee2hip(S:E))
title('Lever Lengths')
legend('bar2hip','knee2hip')

figure(11)
plot(FRAME(S:E),M_HIP(S:E),FRAME(S:E),M_KNEE(S:E))
legend('Hip Moment','Knee Moment')
title('Joint Moments')

STP_THIGH_PROX = -M_HIP .* THIGH_W;
STP_THIGH_DIST = -M_KNEE .* THIGH_W;
STP_TRUNK_PROX = M_HIP .* TRUNK_W;
HIP_POWER = M_HIP .* (TRUNK_W - THIGH_W);

HIP_W = TRUNK_W - THIGH_W;

STP_THIGH_KNEE = trapz(trial_t(S:E),STP_THIGH_DIST(S:E))/1000;
STP_THIGH_HIP = trapz(trial_t(S:E),STP_THIGH_PROX(S:E))/1000;
STP_TRUNK = trapz(trial_t(nTilt:E),STP_TRUNK_PROX(nTilt:E))/1000;
STP_THIGH = STP_THIGH_KNEE + STP_THIGH_HIP;

figure(12)
plot(FRAME(S:E),STP_THIGH_PROX(S:E),FRAME(S:E),STP_THIGH_DIST(S:E),FRAME(S:E),STP_TRUNK_PROX(S:E),FRAME(S:E),HIP_POWER(S:E))
legend('STP THIGH PROX','STP THIGH DIST','STP TRUNK PROX','HIP POWER')
title('Segment Torque Powers')

figure(13)
plot(FRAME(S:E),STP_TRUNK_PROX(S:E),FRAME(S:E),STP_THIGH_PROX(S:E),FRAME(S:E),STP_THIGH_DIST(S:E))
legend('Back/Hamstrings','Glute/Hamstrings','Quads')
title('Muscular Group Contributions')

TOTAL = STP_THIGH + STP_TRUNK;
PROP_THIGH = STP_THIGH/TOTAL;
PROP_TRUNK = STP_TRUNK/TOTAL;
PROP_PROX = STP_THIGH_KNEE/TOTAL;

MAX_HIP_M = max(M_HIP(S:E));
MAX_KNEE_M = max(M_KNEE(S:E));

nHIP_M = MAX_HIP_M/MASS_BAR;
nKNEE_M = MAX_KNEE_M/MASS_BAR;

TOTAL_REP_TIME = trial_t(END) - trial_t(START); 
ECCENTRIC_TIME = trial_t(ECC_END) - trial_t(START);
PAUSE_TIME = trial_t(CON_START) - trial_t(ECC_END);
CONCENTRIC_TIME = trial_t(END) - trial_t(CON_START);

for n = CON_START:END
    if THIGH_W(n + 1) > THIGH_W(n)
        loc1 = i;
        break
    end
    loc2 = find(THIGH_W == min(THIGH_W(CON_START:END)));
end
MIN_CONCENTRIC_VELOCITY = min(THIGH_W(loc1:loc2));

AVERAGE_CONCENTRIC_VELOCITY = sum(THIGH_W(CON_START+1:END))/CONCENTRIC_TIME;
AVERAGE_HIP_POWER = sum(HIP_POWER(CON_START+1:END))/CONCENTRIC_TIME;

if (OFF)
k = 0;
for n=START:END
    field = zeros(401,401);
    origin = [250,201];
    k=k+1;
    
    [knee(1), knee(2),hip(1), hip(2)] = segment_pts_center_rot(L_THIGH,origin(1),origin(2),THIGH_ANG(n)-180);
    [shoulder(1),shoulder(2)] = segment_pts(L_TRUNK,hip(1),hip(2),TRUNK_ANG(n));
    [bar(k,1), bar(k,2)] = bar_pts(shoulder(1),shoulder(2),bar_offset,LIFT);
    
    pt_width = 1;
    field(knee(1)-pt_width:knee(1)+pt_width,knee(2)-pt_width:knee(2)+pt_width) = 1;
    field(hip(1)-pt_width:hip(1)+pt_width,hip(2)-pt_width:hip(2)+pt_width) = 1;
    field(shoulder(1)-pt_width:shoulder(1)+pt_width,shoulder(2)-pt_width:shoulder(2)+pt_width) = 1;
    field(bar(k,1)-pt_width:bar(k,1)+pt_width,bar(k,2)-pt_width:bar(k,2)+pt_width) = 1;
      
       for i=1:99
           [thigh_pt(1),thigh_pt(2)] = segment_pts((-L_THIGH*.01*i),knee(1),...
               knee(2),THIGH_ANG(n)-180);
           field(thigh_pt(1),thigh_pt(2)) = 1;

           [trunk_pt(1),trunk_pt(2)] = segment_pts((L_TRUNK*.01*i),hip(1),...
               hip(2),TRUNK_ANG(n));
           field(trunk_pt(1),trunk_pt(2)) = 1;
       end
    
    figure(99)
    imagesc(field)
    pause(.005)
end
end
end

