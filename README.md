The Masquerade FTC Library.

This README will teach you all you need to know about using this API if you have further questions
you can contact me on telegram @An0nXm0us or email at archishou@gmail.com

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

MasqLinearOpMode

The MasqLinearOpMode implement the MasqRobot, and DashBoard so its not necessary to create a
new MasqRobot and new DashBoard for every opMode Example Implementation

`@Autonomous (name = "AutoTest", group = "Test")
public class AutoTest extends MasqLinearOpMode {
public void runLinearOpMode() throws InterruptedException {
    while (!isStarted()) {
        dashBoard.create("Status", "Initialized");
        dashBoard.create(robot.driveTrain);
        dashBoard.create(robot.imu);
        dashBoard.update();
    }
        waitForStart();
        robot.drive(90, Direction.FORWARD);
        robot.turn(90, Direction.FORWARD);
    }
}
`

