% euler calculation of pitch roll yaw
% this file should be created by spinning in all directions
[FileName,PathName,FilterIndex] = uigetfile({'*.CSV','CSV files (*.CSV)'},'Select file for magnetometer offset');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
FileName
cd(PathName);
M = csvread(FileName, 1, 1);

INER=[];
accel_x = M(:,1);   %X
accel_y = M(:,2);   %Y
accel_z = M(:,3);   %Z

mag_x = M(:,8);  % X
mag_y = M(:,7);  % Y
mag_z = M(:,9);  % Z

magXoffset = (max(mag_x) + min(mag_x)) / 2;
magYoffset = (max(mag_y) + min(mag_y)) / 2;
magZoffset = (max(mag_z) + min(mag_z)) / 2;

% this file is to analyze using offsets previously calculated
[FileName,PathName,FilterIndex] = uigetfile({'*.CSV','CSV files (*.CSV)'},'Select file to analyze motion');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
FileName
cd(PathName);
M = csvread(FileName, 1, 1);

INER=[];
accel_x = M(:,1);   %X
accel_y = M(:,2);   %Y
accel_z = M(:,3);   %Z

mag_x = M(:,8);  % X
mag_y = M(:,7);  % Y
mag_z = M(:,9);  % Z

magXoffset = (max(mag_x) + min(mag_x)) / 2;
magYoffset = (max(mag_y) + min(mag_y)) / 2;
magZoffset = (max(mag_z) + min(mag_z)) / 2;

mag_x = mag_x - magXoffset;
mag_y = mag_y - magYoffset;
mag_z = mag_z - magZoffset;

% Plot showing magnetomer offset correction
% figure(2)
% plot(mag_x, mag_y);
% hold on
% plot(mag_x, mag_z, 'r');

% roll
phi = atan2(accel_y, accel_z); % roll in radians

% de-rotate by roll angle
sinAngle = sin(phi);
cosAngle = cos(phi);
Bfy = (mag_y .* cosAngle) - (mag_z .* sinAngle);
Bz = (mag_y .* sinAngle) + (mag_z .* cosAngle);
Gz = (accel_y .* sinAngle) + (accel_z .* cosAngle);

% theta = pitch angle
theta = atan(- accel_x ./ Gz);
sinAngle = sin(theta);
cosAngle = cos(theta);

% de-rotate by pitch angle theta
Bfx = (mag_x .* cosAngle) + (Bz .* sinAngle);
Bfz = (-mag_x .* sinAngle) + (Bz .* cosAngle);

% Psi = yaw = heading
psi = atan2(-Bfy, Bfx);

pitch = radtodeg(theta);
roll = radtodeg(phi);
yaw = -radtodeg(psi);

figure(1)
plot(pitch)
hold on
plot(roll, 'g')
plot(yaw, 'r')
ylabel('Degrees');
legend('pitch', 'roll', 'yaw')