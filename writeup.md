# CarND-Controls-PID project

## Description of the P,I,D components:

The P component provides the correction proportional to the error. This component enables
fast correction ofthe state towards desired state. P component alone tends to overshoot the
correction, which makes the error trajectory to oscilate around 0.

The D component is proportional to the change of the error from the previous error. This component
flattens error trajectory

The I component is proportional to accumulated error and the goal of this component is to counteract the possible bias,
which can not be overdone by P and D components alone.

## Description of the parameters tune up procedure:

I have started with the following set of the parameters:

Kp = 0.1  Ki = 0.001 Kd = 1.0

The choice was done through manual experiments with several different combinations.
This combination enabled the car to go through the complete track. After that, the parameters
were tuned up using the algorithm described in the lectures. At each iteration I have run the parameters
over 2000 frames (large enough to go through the complete track at least once). I have run the tuneup procedure
for 250 iterations, recording the parameters producing the smallest error.

The final set of the parameters was:

Kp = 0.153802  Kd = 1.15846  Ki= 0.000846829
