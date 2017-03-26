The Masquerade FTC Library.

This README will teach you all you need to know about using this API if you have further questions
you can contact me on telegram @An0nXm0us or email at archishou@gmail.com
*****************************************
Setup

This Document Will Describe How To Use This API.

The first step to use the masq api is to configure your robot to
match the Tank Drive Class. You must have at least two drive motors
and an imu or some sort of gyro to use this system effectively.
First navigate to the tank drive class, Masquerade4997Library -->
Library4997 --> MasqRobot --> MasqRobot. Open the file and begin to
initialize all your hardware in the block of code that currently has
public Motor leftFront = new Motor("leftFront") and so on.
To create a new motor Type public Motor then type the name of the
motor for example leftFront then new Motor( then type the name
of the motor in your config to avoid confusion use the same name
you used in the code so the full line would be
public Motor leftFront = new Motor("leftFront")
In this manner all the sensors can be created with the
exception of the color sensor(scroll to the color section block for more info).
After creating all your hardware get rid of any methods that don't use the
hardware you have obviously you can't use those methods.

*****************************************
MasqAdafruitIMU

Masquerades Adafruit IMU code is very easy to use to create an imu
AdafruitIMU imu = new AdafruitIMU("imu")to get rotation call imu.getHeading();
to get roll call imu.getRoll(); imu.getPitch(); and to put any angle from a scale of
-180 to 180 instead of 0 to 360 call the imu.adjustAngle();


MasqColorSensor
***********************
Using the MasqColorSensor is easy once you understand how to use it.
To begin with start by defining it like a normal MasqHardware Object.
MasqColorSensor beaconColor = new MasqColorSensor("beaconColor", after
this the object requires you to type in the i2c address if you just have
a factory color sensor from modern robotics type 60 and move on, make sure
to set the color sensor as an i2c device in the config. if you have multiple
color sensors or a color sensor that has the i2c address changes continue reading.
i2c addresses are encoded in hexa-decimal thats what the 0x signifys. To decrypt from
hexadecimal multiple the first number after the 0x by 16 and the the a,b,c,d,e,f after
represent 10, 11, 12, 13, 14, 15 add this to the product you get from the first number.
For example if you had 0x3e you take the 3 and multiply it by 16 which is 48 and then
add 14 which is e 62. To find the i2x address of your color sensor you can use modern
robotics core device discovery. the resulting number will be you second argument.
In conclusion if you had a color sensor who's i2c address was 0x40 we would define it
as MasqColorSensor beaconColor = new MasqColorSensor("beaconColor", 64);

Functions:

Just like the normal color sensor you can pull the red blue green
alpha values get the hue and saturation however with this sensor
being i2c we have eliminated the issue where after a while the
enableLed(true); stops working. The best part is the addition of
color number a feature not offered by the FTC SDK that gives accurate
color readings. to use it simply call the function colorSensor.colorNumber();
we have also pre built in isBlue(); isRed(); isWhite(); functions.

***********************

DashBoard

DashBoard is Masquerade's custom telemetry system. It can be accessed
from anywhere in subclasses, parent classes and in normal opmodes.
To create a new DashBoard type DashBoard.getDash; from here you can call the
create method or createSticky method. create will act a normal telemetry
however you don't need to call a different methods for just strings
strings and data and an extra feature is being able to telemetrize MasqHardware.
So the three cases would be dash.create("hello"); dash.create("hello", motor.getRate);
dash.create(robot.imu);, or if you wanted you could get the dash of
the entire robot by doing dash.create(robot);These same arguments can be
applied with createSticky however the arguments in here will be constantly
updated and constantly tracked no need to call the update method.

***********************

MasqLinearOpMode

The MasqLinearOpMode implement the MasqRobot, and DashBoard so its not necessary to create a
new MasqRobot and new DashBoard for every opMode.

********************

MasqSensors

For all other Sensors you can create them just as you
did with all other hardware and whatever data or things
you may need to do just type the name of that hardware
and the method will pop up in auto complete.