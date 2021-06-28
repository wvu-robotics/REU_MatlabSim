function [IMU] = noisy_imu()
%is an accelerometer gyroscope
%   Detailed explanation goes here

IMU = imuSensor('accel-gyro');

IMU.SampleRate = 100;

IMU.Accelerometer.Resolution = .02; %(m/s^2)/ LSB
IMU.Accelerometer.AxesMisalignment = [0,0,0]; %  %skew
IMU.Accelerometer.BiasInstability = [0,0,0]; % m/s^2
IMU.Accelerometer.ConstantBias = [.01, 0,0];% m/s^2
IMU.Accelerometer.NoiseDensity = [.01,.01, 0]; % (m/s2/√Hz)
IMU.Accelerometer.RandomWalk = [.001, .001, 0]; % (m/s2)(√Hz)
IMU.Accelerometer.TemperatureBias = [0,0,0]; % (m/s2)/℃
IMU.Accelerometer.TemperatureScaleFactor = [0,0,0]; %  %/℃

IMU.Gyroscope.Resolution = .02; %(rad/s)/ LSB
IMU.Gyroscope.AxesMisalignment = [0,0,0]; % %skew
IMU.Gyroscope.BiasInstability = [0,0,0]; % rad/s
IMU.Gyroscope.ConstantBias = [.01, 0,0];% rad/s
IMU.Gyroscope.NoiseDensity = [.01,.01, 0]; % (rad/s/√Hz)
IMU.Gyroscope.RandomWalk = [.001, .001, 0]; % (rad/s)(√Hz)
IMU.Gyroscope.TemperatureBias = [0,0,0]; % (rad/s)/℃
IMU.Gyroscope.TemperatureScaleFactor = [0,0,0]; %  %/℃
IMU.Gyroscope.AccelerationBias = [0,0,0];


end

