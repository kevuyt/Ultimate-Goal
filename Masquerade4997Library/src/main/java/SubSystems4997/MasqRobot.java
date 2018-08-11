package SubSystems4997;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqControlSystems.MasqPositionTracker;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
//TODO make MasqRobot abstract to support multiple copies of a robot, for test bot, main bot, so forth
public abstract class MasqRobot {
    public MasqDriveTrain driveTrain;
    public static MasqPositionTracker positionTracker;
    private DashBoard dash = DashBoard.getDash();
    public abstract void mapHardware(HardwareMap hardwareMap);
    private MasqClock timeoutClock = new MasqClock();
    public double angleLeftCover = 0;
    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}
    public void drive(double distance, double speed, Direction direction, double timeOut, int sleepTime) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = positionTracker.getRotation();
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && (clicksRemaining / targetClicks) > 0.05);
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

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeOut, int sleepTime) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = angle;
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
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
            prevAngularError = angularError;
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

    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {


        driveTrain.setClosedLoop(false);
        double targetAngle = positionTracker.imu.adjustAngle(positionTracker.getRotation()) + (direction.value * angle);
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (positionTracker.imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
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
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", positionTracker.getRotation());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
        }
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
        double targetAngle = positionTracker.imu.adjustAngle((direction.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (positionTracker.imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
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
            dash.create("Heading", positionTracker.getRotation());
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

    public void go(double x, double y, int rotation, double speed) {
        double targetHeading = positionTracker.imu.adjustAngle(rotation);
        double xTarget = x - positionTracker.xWheel.getInches();
        double yTarget = y - positionTracker.yWheel.getInches();
        double xError, yError;
        double targetDistance = Math.hypot(xTarget, yTarget), distanceRemaining;
        double power;
        double angle;
        double rotationMultiplier = rotation / Math.abs(rotation);
        double timeChange, angularError, angularIntegral = 0, angularDerivative, powerAdjustment, prevAngularError = 0;
        MasqClock loopTimer = new MasqClock();
        do {
            xError = xTarget - positionTracker.xWheel.getInches();
            yError = yTarget - positionTracker.yWheel.getInches();
            angle = Math.toDegrees(Math.atan2(xError, yError));
            distanceRemaining = Math.hypot(xError, yError);
            power = (distanceRemaining / targetDistance) * speed * MasqUtils.KP.GO_ENCODER;
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = positionTracker.imu.adjustAngle(targetHeading - positionTracker.getRotation());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= rotationMultiplier;
            if (driveTrain instanceof MasqMechanumDriveTrain) setPowerAtAngle(angle, power, powerAdjustment);
         } while (opModeIsActive() && distanceRemaining > 0.5);
        driveTrain.stopDriving();
    }
    public void go(double x, double y, int rotation) {go(x, y, rotation, 0.7);}

    public void stopBlue(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = positionTracker.getRotation();
        while ((!colorSensor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = positionTracker.getRotation();
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
        double targetAngle = positionTracker.getRotation();
        while (!(colorSensor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = positionTracker.getRotation();
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

    public void stop(StopCondition stopCondition, double angle, double speed, Direction direction, double timeOut) {
        driveTrain.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = angle;
        double angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.DRIVE_ANGULAR * power) * angularError + (MasqUtils.KI.DRIVE * angularIntegral) + (MasqUtils.KD.DRIVE * angularDerivative);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setPower(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && stopCondition.stop());
        driveTrain.stopDriving();
    }
    public void stop (StopCondition stopCondition, double angle, double speed, Direction direction) {
        stop(stopCondition, angle, speed, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void stop(StopCondition sensor, double angle, double power) {stop(sensor, angle, power, Direction.FORWARD);}
    public void stop(StopCondition stopCondition, double angle) {stop(stopCondition, angle, 0.5);}
    public void stop(StopCondition sensor){
        stop(sensor, positionTracker.getRotation());
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
    }
    public void TANK(MasqController c){
        double left = c.leftStickX();
        double right = c.rightStickY();
        double leftRate = driveTrain.leftDrive.getVelocity();
        double rightRate = driveTrain.rightDrive.getVelocity();
        double maxRate = MasqUtils.max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setVelocity(right - (rightError * MasqUtils.KP.MOTOR_TELEOP));
        driveTrain.leftDrive.setVelocity(left - (leftError *  MasqUtils.KP.MOTOR_TELEOP));
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
        double max = MasqUtils.max(Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightFront), Math.abs(rightBack));
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
        driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value);
        driveTrain.leftDrive.motor2.setVelocity(leftBack  * direction.value);
        driveTrain.rightDrive.motor1.setVelocity(rightFront  * direction.value);
        driveTrain.rightDrive.motor2.setVelocity(rightBack  * direction.value);
        dash.create("FRONT LEFT: ", driveTrain.leftDrive.motor1.getVelocity());
        dash.create("FRONT RIGHT: ", driveTrain.rightDrive.motor1.getVelocity());
        dash.create("BACK RIGHT: ", driveTrain.rightDrive.motor2.getVelocity());
        dash.create("BACK LEFT: ", driveTrain.leftDrive.motor2.getVelocity());
        dash.update();
    }
    public void MECH(MasqController c, Direction direction) {
        MECH(c, direction, false);
    }
    public void MECH(MasqController c, boolean disabled) {
        MECH(c, Direction.FORWARD, disabled);
    }
    public void MECH(MasqController c) {
        MECH(c, Direction.FORWARD, false);
    }

    public double getDelay() {return FtcRobotControllerActivity.getDelay();}

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
    public void sleep(double time) {
        try {
            Thread.sleep((long) time * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void sleep() {sleep(MasqUtils.DEFAULT_SLEEP_TIME);}

    public void setPowerAtAngle(double angle, double speed, double turnPower) {
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) - turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) - turnPower  * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) + turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.MECH_DRIVE_MULTIPLIER) + turnPower * MasqUtils.MECH_ROTATION_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        driveTrain.leftDrive.motor1.setVelocity(leftFront);
        driveTrain.leftDrive.motor2.setVelocity(leftBack);
        driveTrain.rightDrive.motor1.setVelocity(rightFront);
        driveTrain.rightDrive.motor2.setVelocity(rightBack);
    }
}
