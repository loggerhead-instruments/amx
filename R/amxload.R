# amxload.R: Read in Loggerhead .wav files that were converted from AMX and load into list for tagtools
# Converts raw values to calibrated units
# Assumes all sensors are recorded (accel, mag, gyro, pressure, temperature, RGB light)

# Required packages
library(tuneR)

# load multiple wav files and merge together
# specify path for folder of files to load

# Mac style path
# path = "/Volumes/Dropbox/Dropbox/AMX_test/"

# Windows style path
path = "C:/w/data/humpback/wav"
files <- list.files(path=path, pattern="*.wav", full.names=T, recursive=FALSE)
numfiles = length(files) / 4
INER <- rep(NA,329850 * numfiles)  # Inertial vector
P <- rep(NA, 100 * numfiles)   # Pressure vector
T <- rep(NA, 100 * numfiles)   # Temperature vector

startIMU = 1
endIMU = 1
startPTMP = 1
endPTMP = 1

for (fileName in files){
  print(fileName)
  wavObj <- readWave(fileName)
      
  # add to appropriate dataframe as read in
  if(regexpr('3D', fileName) > 0){
    endIMU = startIMU + length(wavObj@left) - 1
    INER[startIMU:endIMU] <- wavObj@left
    startIMU = endIMU + 1
    inerSrate = wavObj@samp.rate
  }
  if(regexpr('PR', fileName) > 0){
    endPTMP = startPTMP + length(wavObj@left) - 1
    P[startPTMP:endPTMP] <- wavObj@left
    T[startPTMP:endPTMP] <- wavObj@right
    startPTMP = endPTMP + 1
    ptSrate = wavObj@samp.rate
  }
}

# calibrate values
accel_cal=16.0/4096.0;  #16 g/4096 (13 bit ADC)
gyro_cal=500.0/32768.0;  # 500 degrees per second (16-bit ADC)
mag_cal=1.0/1090.0;  #1090 LSB/Gauss

# Inertial headings calibration
n = length(INER)
# OpenTag data aren't stored in NED orientation
# Sign to get NED orientation
# Flip X and Y
# cal_matrix = [M_ACCEL_CAL, -M_ACCEL_CAL, -M_ACCEL_CAL,
#              M_MAG_CAL, -M_MAG_CAL, -M_MAG_CAL,
#              -M_GYRO_CAL, M_GYRO_CAL, -M_GYRO_CAL]

# truncate to divisible by 9 points using modulus
if (n %% 9)  {
  INER = INER[-(n + 1 - (n %% 9)): -n]
}
n = length(INER)
INER = data.frame("accelY" = accel_cal * INER[seq(1, n, 9)],
                  "accelX" = -accel_cal * INER[seq(2, n, 9)],
                  "accelZ" = -accel_cal * INER[seq(3, n, 9)],
                  "magY" = mag_cal * INER[seq(4, n, 9)],
                  "magX" = -mag_cal * INER[seq(5, n, 9)],
                  "magZ" = -mag_cal * INER[seq(6, n, 9)],
                  "gyroY" = -gyro_cal * INER[seq(7, n, 9)],
                  "gyroX" = gyro_cal * INER[seq(8, n, 9)],
                  "gyroZ" = -gyro_cal * INER[seq(9, n, 9)]
)

# Pressure/Temperature
# get surface pressure from first 10 minutes
surfacePressure = min(P[1:600])
mBarPerMeter = 111.377
depth = (surfacePressure - P) / mBarPerMeter

# # Datetime
# startDT = make_datetime(year = year + 2000, month=month, day=mday, hour=hour, min=minute, sec=second, tz="UTC")
# periodS = period / 1000000.0
# periodS[2] = periodS[2] * 2 # *2 because alternates pressure and temperature
# 
# # Pressure/Temp
# n = nrow(PTMP)
# duration = n * periodS[2]   #duration in seconds 
# endDT = startDT + dseconds(duration)
# PTMP$datetime = seq(startDT, endDT, length.out = n)
# 
# # Inertial
# n = nrow(INER)
# duration = n * periodS[1]  #duration in seconds
# endDT = startDT + dseconds(duration)
# INER$datetime = seq(startDT, endDT, length.out = n)