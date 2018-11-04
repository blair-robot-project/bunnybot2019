#Smooths a value while taking its derivative with respect to time.
smoothDerivative <- function(value, timeMillis, n){
  smoothed <- (value[(n+1):length(value)] - value[1:(length(value)-n)])/((timeMillis[(n+1):length(timeMillis)] - timeMillis[1:(length(timeMillis)-n)])/1000);
  return(c(rep(0, ceiling(n/2)), smoothed, rep(0, floor(n/2))));
}

characterize <- function(velFile, accelFile, smoothing = 2){
  vel <- read.csv(velFile)
  accel <- read.csv(accelFile)
  goodVel <- subset(vel, abs(Drive.left_vel) > 0.1 & Drive.left_voltage!=0 & abs(Drive.right_vel) > 0.1 & Drive.right_voltage!=0)
  goodVel$left_accel <- smoothDerivative(goodVel$Drive.left_vel, goodVel$time, smoothing)
  goodVel$right_accel <- smoothDerivative(goodVel$Drive.right_vel, goodVel$time, smoothing)
  accel$left_accel <- smoothDerivative(accel$Drive.left_vel, accel$time, smoothing)
  accel$right_accel <- smoothDerivative(accel$Drive.right_vel, accel$time, smoothing)
  goodAccel <- subset(accel, Drive.left_voltage != 0 & Drive.right_voltage != 0)
  goodAccelLeft <- goodAccel[(which.max(abs(goodAccel$left_accel))+1):length(goodAccel$time),]
  goodAccelRight <- goodAccel[(which.max(abs(goodAccel$right_accel))+1):length(goodAccel$time),]
  combinedLeftVoltage <- c(goodVel$Drive.left_voltage, goodAccelLeft$Drive.left_voltage)
  combinedRightVoltage <- c(goodVel$Drive.right_voltage, goodAccelRight$Drive.right_voltage)
  combinedLeftVel <- c(goodVel$Drive.left_vel, goodAccelLeft$Drive.left_vel)
  combinedRightVel <- c(goodVel$Drive.right_vel, goodAccelRight$Drive.right_vel)
  combinedLeftAccel <- c(goodVel$left_accel, goodAccelLeft$left_accel)
  combinedRightAccel <- c(goodVel$right_accel, goodAccelRight$right_accel)
  plot(goodAccelLeft$time, goodAccelLeft$left_accel)
  plot(goodVel$time, goodVel$Drive.left_voltage)
  plot(goodVel$Drive.left_voltage, goodVel$Drive.left_vel)
  leftModel <- lm(combinedLeftVoltage~combinedLeftVel+combinedLeftAccel)
  rightModel <- lm(combinedRightVoltage~combinedRightVel+combinedRightAccel)
  print(summary(leftModel))
  print(summary(rightModel))
}