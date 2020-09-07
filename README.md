# CameraGimbal
This is the Arduino code for a camera gimbal I designed. The handheld camera gimbal is modelled after the Tarot Professional 5D2 gimbal. It is designed such that it can be assembled and disassembled quickly using PVC pipes and 3D-printed parts.


I used an MPU6050 Accelerometer/Gyroscope module as the input to detect movements and two standard servos as output to counteract movements. I controlled this system by filtering the MPU6050 input with a Kalman filter (LQE) and feeding the processed input into a tuned PID loop, computed on an Arduino Nano. The gimbal is powered by two 18650 cells, producing 8.4V, which is then stepped-down to 6V using an LM2596 Buck Converter.


## More Information Regarding this Project:
If you want to read up on the full description of the project, including the downloadable STL files and a part list, check out the following link: https://grabcad.com/library/3d-printable-handheld-camera-gimbal-1

