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
    O2=[];
end

INER=[];
INER.accel=[];INER.mag=[];INER.gyro=[];
ADC=[];
PTMP=[];
INER_ts=[];
PTMP_ts=[];

for x=1:length(SID_REC)
    cur_sid=(SID_REC(x).nSID) + 1;
    if(SID_SPEC(cur_sid).SID(1)=='A')
        AUDIO=vertcat(AUDIO,SID_REC(x).data);
    end
    if(SID_SPEC(cur_sid).SID(1)=='P')
        PT=vertcat(PT,SID_REC(x).data);
    end
    if(SID_SPEC(cur_sid).SID(1)=='L')
        RGB=vertcat(RGB,SID_REC(x).data);
    end
    if(SID_SPEC(cur_sid).SID(1)=='I')
        IMU=vertcat(IMU,SID_REC(x).data);
        IMU_SID = cur_sid;
    end
        if(SID_SPEC(cur_sid).SID(1)=='O')
        O2=vertcat(O2,SID_REC(x).data);
    end
end

INER.accel.x = IMU(1:9:end) * SID_SPEC(IMU_SID).sensor.cal(1);
INER.accel.y = IMU(2:9:end) * SID_SPEC(IMU_SID).sensor.cal(2);
INER.accel.z = IMU(3:9:end) * SID_SPEC(IMU_SID).sensor.cal(3);

INER.gyro.x = IMU(4:9:end) * SID_SPEC(IMU_SID).sensor.cal(4);
INER.gyro.y = IMU(5:9:end) * SID_SPEC(IMU_SID).sensor.cal(5);
INER.gyro.z = IMU(6:9:end) * SID_SPEC(IMU_SID).sensor.cal(6);

INER.mag.x = IMU(7:9:end) * SID_SPEC(IMU_SID).sensor.cal(7);
INER.mag.y = IMU(8:9:end) * SID_SPEC(IMU_SID).sensor.cal(8);
INER.mag.z = IMU(9:9:end) * SID_SPEC(IMU_SID).sensor.cal(9);

if(length(AUDIO)>0)
    figure(1)
    plot(AUDIO);
end

figure(2)
subplot(3,1,1)
plot(INER.accel.x, 'b');
hold on;
plot(INER.accel.y, 'r');
plot(INER.accel.z, 'g');
title('accelerometer')

subplot(3,1,2)

title('gyroscope')
plot(INER.gyro.x, 'b');
hold on;
plot(INER.gyro.y, 'r');
plot(INER.gyro.z, 'g');

subplot(3,1,3)
plot(INER.mag.x, 'b');
hold on;
plot(INER.mag.y, 'r');
plot(INER.mag.z, 'g');
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


figure(5)
subplot(2,1,1)
plot(O2(1:2:end));
ylabel('Temp');
subplot(2,1,2)
plot(O2(2:2:end));
ylabel('Phase');
title('O2');
