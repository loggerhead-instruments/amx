% amxPlot
if(length(AUDIO)>0)
    figure(1)
    plot(AUDIO);
end

figure(2)
subplot(3,1,1)
plot(INER.accel.x, 'b');
ylabel('g');
hold on;
plot(INER.accel.y, 'r');
ylabel('g');
plot(INER.accel.z, 'g');
ylabel('g');
title('accelerometer')

subplot(3,1,2)
plot(INER.gyro.x, 'b');
title('gyroscope')
ylabel('deg/s');
hold on;
plot(INER.gyro.y, 'r');
ylabel('deg/s');
plot(INER.gyro.z, 'g');
ylabel('deg/s');

subplot(3,1,3)
plot(INER.mag.x, 'b');
ylabel('uT');
hold on;
plot(INER.mag.y, 'r');
ylabel('uT');
plot(INER.mag.z, 'g');
ylabel('uT');
title('magnetometer')

figure(3)
subplot(2,1,1)
plot(PT(1:2:end))
ylabel('Pressure');
subplot(2,1,2)
plot(PT(2:2:end));
ylabel('Temperature');

figure(4)
plot(light.red, 'r');
ylabel('uWpercm^2');
hold on
plot(light.green, 'g');
ylabel('uWpercm^2');
plot(light.blue, 'b');
ylabel('uWpercm^2');
title('Light');

if(length(O2))
    figure(5)
    subplot(3,1,1)
    plot(O2(1:3:end));
    title('O2');
    ylabel('Temp');
    subplot(3,1,2)
    plot(O2(2:3:end));
    ylabel('Phase');
    subplot(3,1,3)
    plot(O2(3:3:end));
    ylabel('Amplitude');
end

