% amxLoad
% loads AMX files into matrices
% modified 7/26/2016

% Surface pressure estimate in mbar for depth calculation
% depth calculation assumes 1 bar = 10 m
surfacepress=1010;

%ans=input('Append? 0=No  1=Yes ');
ans=0;

%% Uncomment to load one file at a time using dialog box
[FileName,PathName,FilterIndex] = uigetfile({'*.amx','AMX files (*.amx)'},'Select an AMX file');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
FileName
cd(PathName);

%% Load file
[DF_HEAD, SID_SPEC, SID_REC]=oAMX(FileName);

if(ans==0)
    AUDIO=[];
    PT=[];
    RGB=[];
    IMU=[];
    amxdata5=[];
end

INER=[];
INER.accel=[];INER.mag=[];INER.gyro=[];
ADC=[];
PTMP=[];
INER_ts=[];
PTMP_ts=[];

for x=1:length(SID_REC)
    cur_sid=(SID_REC(x).nSID) + 1;
    if(cur_sid==1)
        AUDIO=vertcat(AUDIO,SID_REC(x).data);
    end
    if(cur_sid==2)
        PT=vertcat(PT,SID_REC(x).data);
    end
    if(cur_sid==3)
        RGB=vertcat(RGB,SID_REC(x).data);
    end
    if(cur_sid==4)
        IMU=vertcat(IMU,SID_REC(x).data);
    end
        if(cur_sid==5)
        amxdata5=vertcat(amxdata5,SID_REC(x).data);
    end
end

figure(1)
plot(AUDIO);

figure(2)
subplot(3,1,1)
plot(IMU(1:9:end), 'b');
hold on;
plot(IMU(2:9:end), 'r');
plot(IMU(3:9:end), 'g');
title('accelerometer')

subplot(3,1,2)
plot(IMU(4:9:end), 'b');
hold on;
plot(IMU(5:9:end), 'r');
plot(IMU(6:9:end), 'g');
title('gyroscope')

subplot(3,1,3)
plot(IMU(7:9:end), 'b');
hold on;
plot(IMU(8:9:end), 'r');
plot(IMU(9:9:end), 'g');
title('magnetometer')

figure(3)
subplot(2,1,1)
plot(PT(1:2:end))
ylabel('Pressure');
subplot(2,1,2)
plot(PT(2:2:end));
ylabel('Temperature');

figure(4)
plot(RGB(1:3:end), 'r');
hold on
plot(RGB(2:3:end), 'g');
plot(RGB(3:3:end), 'b');
title('Light');


