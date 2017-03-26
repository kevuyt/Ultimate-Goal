The Masquerade FTC Library.

This wiki will teach you all you need to know about using this API if you have further questions
you can contact me on telegram @An0nXm0us or email at archishou@gmail.com

Masquerades Adafruit IMU code is very easy to use to create an imu
AdafruitIMU imu = new AdafruitIMU("imu")to get rotation call imu.getHeading();
to get roll call imu.getRoll(); imu.getPitch(); and to put any angle from a scale of
-180 to 180 instead of 0 to 360 call the imu.adjustAngle();

