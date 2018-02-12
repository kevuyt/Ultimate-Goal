package MasqueradeSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqSensors.MasqAdafruitIMUv2;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqSensor;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.PID_CONSTANTS;
import Library4997.MasqUtilities.Strafe;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import MasqueradeSubSystems.SubSystems.Flipper;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public class MasqRobot implements PID_CONSTANTS {
    public MasqRobot () {}
    public MasqTankDrive driveTrain;
    public MasqMotorSystem intake;
    public MasqMotor lift, relicLift;
    public MasqAdafruitIMUv2 imu;
    public MasqServo blueRotator, redRotator;
    public MasqREVColorSensor jewelColorRed, jewelColorBlue;
    public Flipper flipper;
    public MasqCRServo wheelOne, wheelTwo;
    public MasqServo relicAdjuster;
    public MasqVoltageSensor voltageSensor;
    public MasqServo jewelArmBlue, jewelArmRed, relicGripper;
    public MasqVuforiaBeta vuforia;
    //TODO GET MasqColorSensorV2 up.
    //public MasqMRColorSensor jewelColor;
    HardwareMap hardwareMap;
    private DashBoard dash;
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        dash = DashBoard.getDash();
        wheelOne = new MasqCRServo("wheelOne", this.hardwareMap);
        wheelTwo = new MasqCRServo("wheelTwo", this.hardwareMap);
        vuforia = new MasqVuforiaBeta();
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        flipper = new Flipper(this.hardwareMap);
        blueRotator = new MasqServo("blueRotator", this.hardwareMap);
        redRotator = new MasqServo("redRotator", this.hardwareMap);
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqTankDrive(this.hardwareMap);
        relicAdjuster = new MasqServo("relicAdjuster", this.hardwareMap);
        imu = new MasqAdafruitIMUv2("imuHubOne", this.hardwareMap);
        jewelArmBlue = new MasqServo("jewelArmBlue", this.hardwareMap);
        jewelArmRed = new MasqServo("jewelArmRed", this.hardwareMap);
        jewelColorRed = new MasqREVColorSensor("jewelColorRed", this.hardwareMap);
        jewelColorBlue = new MasqREVColorSensor("jewelColorBlue", this.hardwareMap);
        relicGripper = new MasqServo("relicGripper", this.hardwareMap);
        relicLift = new MasqMotor("relicLift", this.hardwareMap);
        createLimits();
    }

    private MasqClock timeoutClock = new MasqClock();
    public double angleLeftCover = 0;
    private double color = 1;

    public enum AllianceColor {
        BLUE (-1.0),
        RED (+1.0);
        public final double color;
        AllianceColor (double color) {this.color = color;}
    }
    public enum Targets {
        T1 ("T1");
        public final String value;
        Targets(String value) {this.value = value;}
    }
    public void setAllianceColor(AllianceColor allianceColor){this.color = allianceColor.color;}
    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}
    public void drive(double distance, double speed, Direction direction, double timeOut, int sleepTime) {
        //serializer.createFile(new String[]{"Clicks Remaining", "Power", "Angular Error", "Angular Intergral", "Angular Derivative", "Left Power", "Right Power", "Power Adjustment" }, "DRIVEPID");
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getHeading();
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            //serializer.writeData(new Object[]{clicksRemaining, power, angularError, angularIntegral, angularDerivative, leftPower, rightPower, powerAdjustment});
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && (clicksRemaining / targetClicks) > 0.05);
        //serializer.close();
        driveTrain.stopDriving();
        sleep(sleepTime);
    }
    public void drive(double distance, double speed, Direction strafe, double timeOut) {
        drive(distance, speed, strafe, timeOut, 0);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, 3);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void runToPosition(int distance, Direction direction, double speed, double timeOut, int sleepTime) {
        driveTrain.setDistance(distance);
        driveTrain.runToPosition(direction, speed, timeOut);
        sleep(sleepTime);
    }
    public void runToPosition(int distance, Direction direction, double speed, double timeOut) {
        runToPosition(distance, direction, speed, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void runToPosition(int distance, Direction direction, double speed) {
        runToPosition(distance, direction, speed, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void runToPosition(int distance, Direction direction) {
        runToPosition(distance, direction, 0.7);
    }
    public void runToPosition(int distance) {runToPosition(distance, Direction.FORWARD);}

    public void turn(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        //serializer.createFile(new String[]{"Error", "Proprtional", "Intergral", "Derivitive", "Left Power", " Right Power"}, "TURNPID");
        driveTrain.setClosedLoop(false);
        double targetAngle = imu.adjustAngle(imu.getHeading() + (direction.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = imu.adjustAngle(targetAngle - imu.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = imu.adjustAngle(targetAngle - imu.getHeading());
            integral += currentError * tChange;
            derivative = (currentError - prevError) / tChange;
            double errorkp = currentError * kp;
            double integralki = integral * ki;
            double dervitivekd = derivative * kd;
            newPower = (errorkp + integralki + dervitivekd);
            if (Math.abs(newPower) >= 1) {newPower /= Math.abs(newPower);}
            driveTrain.setPower(-newPower * turnPower, newPower * turnPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
            //serializer.writeData(new Object[]{currentError, errorkp, integralki, dervitivekd, -newPower, newPower});
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", imu.getHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
        //serializer.close();
        driveTrain.setPower(0,0);
        sleep(sleepTime);
    }
    public void turn(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, ki, MasqUtils.KD.TURN);
    }
    public void turn(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp) {
        turn(angle, DIRECTION, timeOut, sleepTime, kp, MasqUtils.KI.TURN);
    }
    public void turn(double angle, Direction DIRECTION, double timeOut, int sleepTime) {
        turn(angle, DIRECTION, timeOut, sleepTime, MasqUtils.KP.TURN);
    }
    public void turn(double angle, Direction DIRECTION, double timeout)  {
        turn(angle, DIRECTION, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turn(double angle, Direction DIRECTION)  {turn(angle, DIRECTION, MasqUtils.DEFAULT_TIMEOUT);}

    public void stopBlue(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while ((!colorSensor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error *  MasqUtils.KP.DRIVE_ANGULAR;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            dash.create("Heading", heading);
            dash.create("Blue Val", colorSensor.colorNumber());
        }
        driveTrain.stopDriving();
    }
    public void stopBlue (MasqColorSensor colorSensor, double power){
        stopBlue(colorSensor, power, Direction.BACKWARD);
    }
    public void stopBlue (MasqColorSensor colorSensor){stopBlue(colorSensor, 0.5);}

    public void stopRed(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        while (!(colorSensor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * MasqUtils.KP.DRIVE_ANGULAR;
            newPower = newPower - (errorkp * Direction.value);
            driveTrain.setPowerLeft(newPower * Direction.value);
            driveTrain.setPowerRight(power * Direction.value);
            dash.create("Heading", heading);
            dash.create("red Val", colorSensor.colorNumber());
        }
        driveTrain.stopDriving();
    }
    public void stopRed (MasqColorSensor colorSensor, double power){
        stopRed(colorSensor, power, Direction.BACKWARD);
    }
    public void stopRed (MasqColorSensor colorSensor){stopRed(colorSensor, 0.5);}

    public void stop(MasqSensor sensor, double speed, Direction Direction) {
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        driveTrain.setClosedLoop(true);
        double targetAngle = imu.getHeading();
        double  angularError = imu.adjustAngle(targetAngle - imu.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, leftPower, rightPower, maxPower, timeChange, power;
        do {
            power = Direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getHeading());
            angularIntegral = angularIntegral + angularError * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power + .01) * angularError +
                    MasqUtils.KI.DRIVE * angularIntegral + MasqUtils.KD.DRIVE * angularDerivative;
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= Direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ",angularError);
            dash.update();
        } while (opModeIsActive() && sensor.stop());
        driveTrain.stopDriving();
    }
    public void stop(MasqSensor sensor, double power) {stop(sensor, power, Direction.FORWARD);}
    public void stop(MasqSensor sensor){
        stop(sensor, 0.5);
    }

    public void strafe(int distance, Strafe direction, double speed, double timeOut, double sleepTime) {
        driveTrain.setClosedLoop(false);
        driveTrain.resetEncoders();
        double targetClicks =(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double power;
        MasqClock timeoutTimer = new MasqClock();
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = (clicksRemaining / targetClicks) * speed;
            driveTrain.leftDrive.motor1.setPower(power * direction.value[0]);
            driveTrain.leftDrive.motor2.setPower(power * direction.value[3]);
            driveTrain.rightDrive.motor1.setPower(power * direction.value[1]);
            driveTrain.rightDrive.motor2.setPower(power * direction.value[2]);
            dash.create("TARGET CLICKS: " + targetClicks);
            dash.create("CLICKS REMAINING: " + clicksRemaining);
            dash.update();
        } while (opModeIsActive()  && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        sleep(sleepTime);
    }
    public void strafe(int distance, Strafe direction, double speed, double timeOut) {
        strafe(distance, direction, speed, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void strafe(int distance, Strafe direction, double speed) {
        strafe(distance, direction, speed, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void strafe(int distance, Strafe direction) {
        strafe(distance, direction, 0.7);
    }

    public void NFS(MasqController c) {
        float move = c.leftStickY();
        float turn = c.rightStickX();
        double left = move - turn;
        double right = move + turn;
        left *= -.6;
        right *= -.6;
        if (c.leftBumper()) {
            left /= 2;
            right /= 2;
        }
        if(left > 1.0) {
            left /= left;
            right /= left;
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        else if (right > 1.0) {
            left /= right;
            right /= right;
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        else {
            driveTrain.setPowerLeft(left);
            driveTrain.setPowerRight(right);
        }
        voltageSensor.update();

    }
    public void MECH(MasqController c, Direction direction) {
        double x = -c.leftStickY();
        double y = c.leftStickX();
        double angle = Math.atan2(y, x);
        double adjustedAngle = angle + Math.PI/4;
        double multiplier = 1.4;
        double speedMagnitude = Math.hypot(x, y);
        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX() * multiplier;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) + c.rightStickX()  * multiplier;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * multiplier) - c.rightStickX() * multiplier;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        if (c.leftTriggerPressed()) {
            leftFront /= 3;
            leftBack /= 3;
            rightFront /= 3;
            rightBack /= 3;
        }
        driveTrain.leftDrive.motor1.setPower(leftFront * direction.value);
        driveTrain.leftDrive.motor2.setPower(leftBack  * direction.value);
        driveTrain.rightDrive.motor1.setPower(rightFront  * direction.value);
        driveTrain.rightDrive.motor2.setPower(rightBack  * direction.value);
        dash.create("FRONT LEFT: ", leftFront);
        dash.create("FRONT RIGHT: ", rightFront);
        dash.create("BACK RIGHT: ", rightBack);
        dash.create("BACK LEFT: ", leftBack);
        dash.update();
    }
    public void TANK(MasqController c){
        double left = c.leftStickX();
        double right = c.rightStickY();
        double leftRate = driveTrain.leftDrive.getRate();
        double rightRate = driveTrain.rightDrive.getRate();
        double maxRate = Math.max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setPower(right - (rightError * MasqUtils.KP.MOTOR_TELEOP));
        driveTrain.leftDrive.setPower(left - (leftError *  MasqUtils.KP.MOTOR_TELEOP));
        voltageSensor.update();
    }

    public int getDelta (double initial, Direction direction) {
        return (int) (initial- (imu.getHeading() * direction.value));
    }
    public double getVoltage() {return voltageSensor.getVoltage();}
    public double getDelay() {return FtcRobotControllerActivity.getDelay();}

    public void waitForVuMark() {
        timeoutClock.reset();
        while (MasqUtils.VuMark.isUnKnown(vuforia.getVuMark()) &&
                !timeoutClock.elapsedTime(2, MasqClock.Resolution.SECONDS)){
            dash.create(vuforia.getVuMark());
            dash.update();
        }
        dash.create(vuforia.getVuMark());
        dash.update();
    }
    public void initializeTeleop(){
        driveTrain.setKp(MasqUtils.KP.MOTOR_TELEOP);
        driveTrain.setKi(MasqUtils.KI.MOTOR_TELEOP);
        driveTrain.setKp(MasqUtils.KD.MOTOR_TELEOP);
    }
    public void initializeAutonomous(){
        driveTrain.setKp(MasqUtils.KP.MOTOR_AUTONOMOUS);
        driveTrain.setKi(MasqUtils.KI.MOTOR_AUTONOMOUS);
        driveTrain.setKp(MasqUtils.KD.MOTOR_AUTONOMOUS);
    }
    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {sleep(MasqUtils.DEFAULT_SLEEP_TIME);}

    public void initializeServos() {
        jewelArmBlue.setPosition(0.26);
        jewelArmRed.setPosition(0.39);
    }
    public void createLimits () {
        //bottomIntake.setLimit(bottomLimit);
    }
}
