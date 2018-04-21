package SubSystems4997;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqMotors.MasqEncoder;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.PID_CONSTANTS;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqUtilities.Strafe;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.SubSystems.Flipper;
import SubSystems4997.SubSystems.Gripper;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public class MasqRobot implements PID_CONSTANTS {
    public MasqRobot () {}
    public MasqTankDrive driveTrain;
    public MasqMotorSystem intake;
    public MasqMotor lift, relicLift;
    public MasqAdafruitIMU imu;
    public MasqServo redRotator;
    public MasqREVColorSensor jewelColorRed;
    public Flipper flipper;
    public Gripper gripper;
    public MasqServo relicAdjuster;
    public MasqVoltageSensor voltageSensor;
    public MasqEncoder yWheel;
    public MasqServo jewelArmRed, relicGripper;
    public MasqVuforiaBeta vuforia;
    public MasqREVColorSensor singleBlock, doubleBlock, redLineDetector, blueLineDetector;
    HardwareMap hardwareMap;
    private int yTarget;
    private DashBoard dash;
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        dash = DashBoard.getDash();
        vuforia = new MasqVuforiaBeta();
        blueLineDetector = new MasqREVColorSensor("blueLineDetector", this.hardwareMap);
        redLineDetector = new MasqREVColorSensor("redLineDetector", this.hardwareMap);
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", this.hardwareMap);
        doubleBlock = new MasqREVColorSensor("doubleBlock", this.hardwareMap);
        singleBlock = new MasqREVColorSensor("singleBlock", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        flipper = new Flipper(this.hardwareMap);
        redRotator = new MasqServo("redRotator", this.hardwareMap);
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqTankDrive(this.hardwareMap);
        relicAdjuster = new MasqServo("relicAdjuster", this.hardwareMap);
        imu = new MasqAdafruitIMU("imuHubOne", this.hardwareMap);
        jewelArmRed = new MasqServo("jewelArmRed", this.hardwareMap);
        jewelColorRed = new MasqREVColorSensor("jewelColorRed", this.hardwareMap);
        relicGripper = new MasqServo("relicGripper", this.hardwareMap);
        relicLift = new MasqMotor("relicLift", this.hardwareMap);
        yWheel = new MasqEncoder(relicLift);
        gripper = flipper.getGripper();
        lift.setClosedLoop(false);
    }

    private MasqClock timeoutClock = new MasqClock();
    public double angleLeftCover = 0;
    private double color = 1;
    private double imuZero = 0;
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
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.getAbsoluteHeading();
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
        drive(distance, speed, strafe, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsolute(StopCondition stopCondition, double distance, int angle, double speed, Direction direction, double timeOut, int sleepTime) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = angle;
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.05) || stopCondition.stop());
        //serializer.close();
        driveTrain.stopDriving();
        sleep(sleepTime);
    }
    public void driveAbsolute(StopCondition stopCondition, double distance, int angle, double speed, Direction strafe, double timeOut) {
        driveAbsolute(stopCondition, distance, angle, speed, strafe, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveAbsolute(StopCondition stopCondition, double distance, int angle, double speed, Direction strafe) {
        driveAbsolute(stopCondition, distance, angle, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void driveAbsolute(StopCondition stopCondition, double distance, int angle, double speed){driveAbsolute(stopCondition, distance, angle, speed, Direction.FORWARD);}
    public void driveAbsolute(StopCondition stopCondition, double distance, int angle) {driveAbsolute(stopCondition, distance, angle, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeOut, int sleepTime) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = angle;
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.05));
        //serializer.close();
        driveTrain.stopDriving();
        sleep(sleepTime);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe, double timeOut) {
        driveAbsoluteAngle(distance, angle, speed, strafe, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe) {
        driveAbsoluteAngle(distance, angle, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed){driveAbsoluteAngle(distance, angle, speed, Direction.FORWARD);}
    public void driveAbsoluteAngle(double distance, int angle) {driveAbsoluteAngle(distance, angle, 0.5);}

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

    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        //serializer.createFile(new String[]{"Error", "Proprtional", "Intergral", "Derivitive", "Left Power", " Right Power"}, "TURNPID");
        driveTrain.setClosedLoop(false);
        double targetAngle = imu.adjustAngle(imu.getAbsoluteHeading() + (direction.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
            currentError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
            dash.create("Heading", imu.getAbsoluteHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
        //serializer.close();
        driveTrain.setPower(0,0);
        sleep(sleepTime);
    }
    public void turnRelative(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki) {
        turnRelative(angle, DIRECTION, timeOut, sleepTime, kp, ki, MasqUtils.KD.TURN);
    }
    public void turnRelative(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp) {
        turnRelative(angle, DIRECTION, timeOut, sleepTime, kp, MasqUtils.KI.TURN);
    }
    public void turnRelative(double angle, Direction DIRECTION, double timeOut, int sleepTime) {
        turnRelative(angle, DIRECTION, timeOut, sleepTime, MasqUtils.KP.TURN);
    }
    public void turnRelative(double angle, Direction DIRECTION, double timeout)  {
        turnRelative(angle, DIRECTION, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction DIRECTION)  {
        turnRelative(angle, DIRECTION, MasqUtils.DEFAULT_TIMEOUT);}

    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        driveTrain.setClosedLoop(false);
        double targetAngle = imu.adjustAngle((direction.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
            currentError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
            dash.create("LEFT POWER: ", -newPower );
            dash.create("RIGHT POWER: " ,newPower);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", imu.getAbsoluteHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
        //serializer.close();
        driveTrain.setPower(0,0);
        sleep(sleepTime);
    }
    public void turnAbsolute(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp, double ki) {
        turnAbsolute(angle, DIRECTION, timeOut, sleepTime, kp, ki, MasqUtils.KD.TURN);
    }
    public void turnAbsolute(double angle, Direction DIRECTION, double timeOut, int sleepTime, double kp) {
        turnAbsolute(angle, DIRECTION, timeOut, sleepTime, kp, MasqUtils.KI.TURN);
    }
    public void turnAbsolute(double angle, Direction DIRECTION, double timeOut, int sleepTime) {
        turnAbsolute(angle, DIRECTION, timeOut, sleepTime, MasqUtils.KP.TURN);
    }
    public void turnAbsolute(double angle, Direction DIRECTION, double timeout)  {
        turnAbsolute(angle, DIRECTION, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle, Direction DIRECTION)  {turnAbsolute(angle, DIRECTION, MasqUtils.DEFAULT_TIMEOUT);}

    public void go(double speed, int x, int y, double rotation, Direction rotationDirection) {
        //Positive is LEFT
        driveTrain.setClosedLoop(false);
        double xClicks = (int)(x * MasqUtils.CLICKS_PER_INCH);
        double yClicks = (y * yWheel.getClicksPerInch());
        rotation *= rotationDirection.value;
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        imu.reset();
        driveTrain.resetEncoders();
        yWheel.resetEncoder();
        double driveAngle;
        double targetAngle = imu.adjustAngle((rotation));
        double xClicksRemaining, yClicksRemaining, yInches, xInches;
        double angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, angularCorrectionPowerAdjustment, power, timeChange;
        do {
            loopTimer.reset();
            xClicksRemaining = (int) (xClicks - Math.abs(driveTrain.getCurrentPosition()));
            yClicksRemaining = (int) (yClicks - Math.abs(yWheel.getPosition()));
            xInches = driveTrain.getCurrentPosition() / MasqUtils.CLICKS_PER_INCH;
            yInches = -yWheel.getPosition() / yWheel.getClicksPerInch();
            driveAngle = -Math.toDegrees(Math.atan2(y - yInches, x - xInches));
            power = ((xClicksRemaining / xClicks) * MasqUtils.KP.GO_ENCODER) * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            angularCorrectionPowerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            angularCorrectionPowerAdjustment = Range.clip(angularCorrectionPowerAdjustment, -1.0, +1.0);
            angularCorrectionPowerAdjustment *= rotationDirection.value;
            double turnMultiplier = .4;
            double driveMultiplier = 1.4;
            double adjustedAngle = Math.toRadians(driveAngle) + (Math.PI/4);
            double leftFront = (Math.sin(adjustedAngle) * power * driveMultiplier) - angularCorrectionPowerAdjustment * turnMultiplier;
            double leftBack = (Math.cos(adjustedAngle) * power * driveMultiplier) - angularCorrectionPowerAdjustment * turnMultiplier;
            double rightFront = (Math.cos(adjustedAngle) * power * driveMultiplier) + angularCorrectionPowerAdjustment * turnMultiplier;
            double rightBack = (Math.sin(adjustedAngle) * power * driveMultiplier) + angularCorrectionPowerAdjustment  * turnMultiplier;
            dash.create("xInchesRemaining: ", xInches);
            dash.create("yInches: ", yInches);
            dash.create("Angle ", driveAngle);
            dash.create("BACK LEFT: ", leftBack);
            dash.update();
            driveTrain.leftDrive.motor1.setPower(leftFront);
            driveTrain.leftDrive.motor2.setPower(leftBack);
            driveTrain.rightDrive.motor1.setPower(rightFront);
            driveTrain.rightDrive.motor2.setPower(rightBack);
            prevAngularError = angularError;
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(2, MasqClock.Resolution.SECONDS) && (((xClicksRemaining / xClicks) > 0.05) || (yClicksRemaining / yClicks) > 0.05));
    }
    public void go (StopCondition condition, double speed, int x, int y, double rotation, Direction rotationDirection) {
        driveTrain.setClosedLoop(false);
        double xClicks = (int)(x * MasqUtils.CLICKS_PER_INCH);
        rotation *= rotationDirection.value;
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        imu.reset();
        driveTrain.resetEncoders();
        yWheel.resetEncoder();
        double driveAngle;
        double targetAngle = imu.adjustAngle((rotation));
        double xClicksRemaining, yInches, xInches;
        double angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, angularCorrectionPowerAdjustment, power, timeChange;
        do {
            loopTimer.reset();
            xClicksRemaining = (int) (xClicks - Math.abs(driveTrain.getCurrentPosition()));
            xInches = driveTrain.getCurrentPosition() / MasqUtils.CLICKS_PER_INCH;
            yInches = -yWheel.getPosition() / yWheel.getClicksPerInch();
            driveAngle = -Math.toDegrees(Math.atan2(y - yInches, x - xInches));
            power = ((xClicksRemaining / xClicks) * MasqUtils.KP.GO_ENCODER) * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            angularCorrectionPowerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            angularCorrectionPowerAdjustment = Range.clip(angularCorrectionPowerAdjustment, -1.0, +1.0);
            angularCorrectionPowerAdjustment *= rotationDirection.value;
            double turnMultiplier = .4;
            double driveMultiplier = 1.4;
            double adjustedAngle = Math.toRadians(driveAngle) + (Math.PI/4);
            double leftFront = (Math.sin(adjustedAngle) * power * driveMultiplier) - angularCorrectionPowerAdjustment * turnMultiplier;
            double leftBack = (Math.cos(adjustedAngle) * power * driveMultiplier) - angularCorrectionPowerAdjustment * turnMultiplier;
            double rightFront = (Math.cos(adjustedAngle) * power * driveMultiplier) + angularCorrectionPowerAdjustment * turnMultiplier;
            double rightBack = (Math.sin(adjustedAngle) * power * driveMultiplier) + angularCorrectionPowerAdjustment  * turnMultiplier;
            dash.create("xInchesRemaining: ", xInches);
            dash.create("yInches: ", yInches);
            dash.create("Angle ", driveAngle);
            dash.create("BACK LEFT: ", leftBack);
            dash.update();
            driveTrain.leftDrive.motor1.setPower(leftFront);
            driveTrain.leftDrive.motor2.setPower(leftBack);
            driveTrain.rightDrive.motor1.setPower(rightFront);
            driveTrain.rightDrive.motor2.setPower(rightBack);
            prevAngularError = angularError;
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(2, MasqClock.Resolution.SECONDS) && condition.stop());
    }

    public void go (int x, int drivingAngle, Direction driveDirection, int rotation, Direction direction, double speed, int timeOut, int sleepTime) {
        rotation *= direction.value;
        drivingAngle *= -driveDirection.value;
        driveTrain.setClosedLoop(false);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.adjustAngle((rotation));
        double targetClicks = (int)(x * MasqUtils.CLICKS_PER_INCH);
        double multiplier = 1.4;
        double rotationMultiplier = .4;
        double clicksRemaining;
        double angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, angularCorrectionPowerAdjustment, power, timeChange;
        do {
            loopTimer.reset();
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.rightDrive.motor1.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.GO_ENCODER) * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            angularCorrectionPowerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            angularCorrectionPowerAdjustment = Range.clip(angularCorrectionPowerAdjustment, -1.0, +1.0);
            angularCorrectionPowerAdjustment *= -direction.value;
            double adjustedAngle = Math.toRadians(drivingAngle) + (Math.PI/4);
            double leftFront = (Math.sin(adjustedAngle) * power * multiplier) - angularCorrectionPowerAdjustment * rotationMultiplier;
            double leftBack = (Math.cos(adjustedAngle) * power * multiplier) - angularCorrectionPowerAdjustment * rotationMultiplier;
            double rightFront = (Math.cos(adjustedAngle) * power * multiplier) + angularCorrectionPowerAdjustment * rotationMultiplier;
            double rightBack = (Math.sin(adjustedAngle) * power * multiplier) + angularCorrectionPowerAdjustment * rotationMultiplier;
            dash.create("Power: ", power);
            dash.create("targetClicks", targetClicks);
            dash.create("getCurrentPosition", driveTrain.rightDrive.motor1.getCurrentPosition());
            dash.create("clicksRemaining", clicksRemaining);
            dash.create("Angular Error ", angularError);
            dash.update();
            driveTrain.leftDrive.motor1.setPower(leftFront);
            driveTrain.leftDrive.motor2.setPower(leftBack);
            driveTrain.rightDrive.motor1.setPower(rightFront);
            driveTrain.rightDrive.motor2.setPower(rightBack);
            prevAngularError = angularError;
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && (clicksRemaining / targetClicks) > 0.1);
        sleep(sleepTime);
    }
    public void go (int x, int drivingAngle, Direction driveDirection, int rotation, Direction direction) {
        go(x, drivingAngle, driveDirection, rotation, direction, .7, 2, 1000);
    }

    public void go (StopCondition condition, int drivingAngle, Direction driveDirection, int rotation, Direction direction, double speed, int timeOut, int sleepTime) {
        rotation *= direction.value;
        drivingAngle *= -driveDirection.value;
        driveTrain.setClosedLoop(false);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = imu.adjustAngle((rotation));
        double multiplier = 1.4;
        double rotationMultiplier = .4;
        double angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, angularCorrectionPowerAdjustment, power, timeChange;
        while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && condition.stop()) {
            loopTimer.reset();
            power = (speed * MasqUtils.KP.GO_ENCODER) * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            angularCorrectionPowerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            angularCorrectionPowerAdjustment = Range.clip(angularCorrectionPowerAdjustment, -1.0, +1.0);
            angularCorrectionPowerAdjustment *= -direction.value;
            double adjustedAngle = Math.toRadians(drivingAngle) + (Math.PI/4);
            double leftFront = (Math.sin(adjustedAngle) * power * multiplier) - angularCorrectionPowerAdjustment * rotationMultiplier;
            double leftBack = (Math.cos(adjustedAngle) * power * multiplier) - angularCorrectionPowerAdjustment * rotationMultiplier;
            double rightFront = (Math.cos(adjustedAngle) * power * multiplier) + angularCorrectionPowerAdjustment * rotationMultiplier;
            double rightBack = (Math.sin(adjustedAngle) * power * multiplier) + angularCorrectionPowerAdjustment * rotationMultiplier;
            dash.create("Angle: ", drivingAngle);
            dash.create("Angular Error ", angularError);
            dash.update();
            driveTrain.leftDrive.motor1.setPower(leftFront);
            driveTrain.leftDrive.motor2.setPower(leftBack);
            driveTrain.rightDrive.motor1.setPower(rightFront);
            driveTrain.rightDrive.motor2.setPower(rightBack);
            prevAngularError = angularError;
        }
        sleep(sleepTime);
    }
    public void go (StopCondition condition, int drivingAngle, Direction driveDirection, int rotation, Direction direction) {
        go(condition, drivingAngle, driveDirection, rotation, direction, .5, 3, 1000);
    }

    public void stopBlue(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getAbsoluteHeading();
        while ((!colorSensor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = imu.getAbsoluteHeading();
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

    public void strafeToY (double speed) {
        MasqClock clock = new MasqClock();
        double clicksRemaining = 0;
        clock.reset();
        if (yTarget < -yWheel.getPosition()) {
            while ((yTarget < -yWheel.getPosition()) && opModeIsActive() && !clock.elapsedTime(1, MasqClock.Resolution.SECONDS) && opModeIsActive()) {
                driveTrain.setPowerAtAngle(-90, speed, 0);
            }
            driveTrain.stopDriving();
        }
        if (yTarget > -yWheel.getPosition()) {
            while ((yTarget > -yWheel.getPosition()) && opModeIsActive() && !clock.elapsedTime(1, MasqClock.Resolution.SECONDS)) {
                driveTrain.setPowerAtAngle(90, speed, 0);
            }
            driveTrain.stopDriving();
        }
    }

    public void stopRed(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getAbsoluteHeading();
        while (!(colorSensor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getAbsoluteHeading();
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

    public void stop(StopCondition stopCondition, int angle, double speed, Direction direction, double timeOut) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = angle;
        double angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getAbsoluteHeading());
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
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(2, MasqClock.Resolution.SECONDS) && stopCondition.stop());
        driveTrain.stopDriving();
    }
    public void stop (StopCondition stopCondition, int angle, double speed, Direction direction) {
        stop(stopCondition, angle, speed, direction, 2);
    }
    public void stop(StopCondition sensor, double speed, Direction Direction) {
        MasqClock loopTimer = new MasqClock();
        MasqClock clock = new MasqClock();
        driveTrain.resetEncoders();
        driveTrain.setClosedLoop(true);
        double targetAngle = imu.getRelativeYaw();
        double  angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, leftPower, rightPower, maxPower, timeChange, power;
        do {
            power = Direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = imu.adjustAngle(targetAngle - imu.getRelativeYaw());
            angularIntegral = angularIntegral + angularError * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power + .01) * angularError +
                    MasqUtils.KI.DRIVE * angularIntegral + MasqUtils.KD.DRIVE * angularDerivative;
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= 0;
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
        } while (opModeIsActive() && sensor.stop() && !clock.elapsedTime(2, MasqClock.Resolution.SECONDS));
        driveTrain.stopDriving();
    }
    public void stop(StopCondition sensor, double power) {stop(sensor, power, Direction.FORWARD);}
    public void stop(StopCondition sensor){
        stop(sensor, 0.5);
    }

    public void strafe(int distance, Strafe direction, double speed, double timeOut, double sleepTime) {
        driveTrain.resetEncoders();
        double targetClicks = (distance * MasqUtils.CLICKS_PER_INCH);
        double power = speed;
        MasqClock timeoutTimer = new MasqClock();
        do {
            driveTrain.leftDrive.motor1.setPower(power * direction.value[0]);
            driveTrain.leftDrive.motor2.setPower(power * direction.value[3]);
            driveTrain.rightDrive.motor1.setPower(power * direction.value[1]);
            driveTrain.rightDrive.motor2.setPower(power * direction.value[2]);
            dash.create("Position: ", yWheel.getPosition());
            dash.create("TARGET CLICKS: " + targetClicks);
            dash.create("POWER: ", power);
            dash.update();
        } while (opModeIsActive()  && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS));
        sleep(sleepTime);
    }
    public void strafe(int distance, Strafe direction, double speed, double timeOut) {
        strafe(distance, direction, speed, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void strafe(int distance, Strafe direction, double speed) {
        strafe(distance, direction, speed, 3);
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
    public void MECH(MasqController c, Direction direction, boolean disabled) {
        double x = -c.leftStickY();
        double y = c.leftStickX();
        double xR = - c.rightStickX();
        double angle = Math.atan2(y, x);
        double adjustedAngle = angle + Math.PI/4;
        double speedMultiplier = 1.4;
        double turnMultiplier = 1.4;
        if (c.leftBumper()) xR = 0;
        double speedMagnitude = Math.hypot(x, y);
        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) - xR * turnMultiplier * direction.value;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) - xR  * turnMultiplier * direction.value;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;
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
        if (disabled) {
            leftBack = 0;
            leftFront = 0;
            rightBack = 0;
            rightFront = 0;
        }
        driveTrain.leftDrive.motor1.setPower(leftFront * direction.value);
        driveTrain.leftDrive.motor2.setPower(leftBack  * direction.value);
        driveTrain.rightDrive.motor1.setPower(rightFront  * direction.value);
        driveTrain.rightDrive.motor2.setPower(rightBack  * direction.value);
        dash.create("FRONT LEFT: ", driveTrain.leftDrive.motor1.getVelocity());
        dash.create("FRONT RIGHT: ", driveTrain.rightDrive.motor1.getVelocity());
        dash.create("BACK RIGHT: ", driveTrain.rightDrive.motor2.getVelocity());
        dash.create("BACK LEFT: ", driveTrain.leftDrive.motor2.getVelocity());
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

    public void setYTarget(double yTarget) {
        this.yTarget = (int) yTarget;
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
        jewelArmRed.setPosition(0.39);
    }
}
