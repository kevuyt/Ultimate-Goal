package SubSystems4997;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqVector;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqColorSensor;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
public abstract class MasqRobot {
    public MasqDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    public DashBoard dash;
    public abstract void mapHardware(HardwareMap hardwareMap);
    private MasqClock timeoutClock = new MasqClock();
    public double angleLeftCover = 0;
    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}
    public void drive(double distance, double speed, Direction direction, double timeOut, int sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
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

        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError = tracker.imu.adjustAngle((double) angle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE_ENCODER) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = tracker.imu.adjustAngle((double) angle - tracker.getHeading());
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
        double targetAngle = tracker.imu.adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (tracker.imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
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
            dash.create("Heading", tracker.getHeading());
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
    public void turnRelative(double angle, Direction DIRECTION, double timeout) {
        turnRelative(angle, DIRECTION, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction DIRECTION)  {
        turnRelative(angle, DIRECTION, MasqUtils.DEFAULT_TIMEOUT);}

    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        double targetAngle = tracker.imu.adjustAngle((direction.value * angle));
        double acceptableError = .5;
        double turnPower = .4;
        double currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (tracker.imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
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
            dash.create("Heading", tracker.getHeading());
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

    public void stopBlue(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = tracker.getHeading
                ();
        while ((!colorSensor.isBlue()) && opModeIsActive()){
            double newPower = power;
            double heading = tracker.getHeading
                    ();
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
    public void stopBlue (MasqColorSensor colorSensor) {stopBlue(colorSensor, 0.5);}

    public void stopRed(MasqColorSensor colorSensor, double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = tracker.getHeading();
        while (!(colorSensor.isRed()) && opModeIsActive()) {
            double newPower = power;
            double heading = tracker.getHeading();
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
    public void stopRed (MasqColorSensor colorSensor) {stopRed(colorSensor, 0.5);}

    public void stop(StopCondition stopCondition, double angle, double speed, Direction direction, double timeOut) {

        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double angularError = tracker.imu.adjustAngle(angle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = tracker.imu.adjustAngle(angle - tracker.getHeading
                    ());
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
        stop(sensor, tracker.getHeading());
    }

    public void go(double x, double y) {
        double basePower, angleAdjustment, yAdjustment;
        double leftPower, rightPower;
        double yError, yIntegral = 0, yDeravitive, yAnglePrev = 0;
        double angleError, angleIntegral = 0, angleDerivative, angleErrorPrev = 0;
        double timeCurrent, timeChange, timePrev = 0;
        MasqVector targetVector = new MasqVector(x, y);
        MasqVector currentPoint = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        while (currentPoint.distanceToVector(targetVector) > 0.1) {
            timeCurrent = System.nanoTime();
            timeChange = timeCurrent - timePrev;
            timeChange /= 1e9;
            basePower = currentPoint.distanceToVector(targetVector) / targetVector.getMagnitude();
            currentPoint.setX(tracker.getGlobalX());
            currentPoint.setY(tracker.getGlobalY());
            angleError = tracker.imu.adjustAngle(targetVector.getDirection() - tracker.getHeading());
            angleIntegral += (angleError * timeChange);
            angleDerivative = (angleError - angleErrorPrev) / timeChange;
            angleAdjustment = (angleError * MasqUtils.KP.GO_ANGLE * basePower) +
                    (angleIntegral * MasqUtils.KI.GO_ANGLE) + (angleDerivative * MasqUtils.KD.GO_ANGLE);
            leftPower = basePower - angleAdjustment;
            rightPower = basePower + angleAdjustment;
            driveTrain.setPower(leftPower, rightPower);
            tracker.updateSystem();
            timePrev = timeCurrent;
            angleErrorPrev = angleError;
        }
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
        }
        else if (right > 1.0) {
            left /= right;
            right /= right;
        }
        driveTrain.setPowerLeft(left);
        driveTrain.setPowerRight(right);
    }
    public void TANK(MasqController c) {
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
        int disable = 1;
        if (disabled) disable = 0;
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
        driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value * disable);
        driveTrain.leftDrive.motor2.setVelocity(leftBack  * direction.value * disable);
        driveTrain.rightDrive.motor1.setVelocity(rightFront * direction.value * disable);
        driveTrain.rightDrive.motor2.setVelocity(rightBack * direction.value * disable);
        dash.create("FRONT LEFT: ", driveTrain.leftDrive.motor1.getVelocity());
        dash.create("FRONT RIGHT: ", driveTrain.rightDrive.motor1.getVelocity());
        dash.create("BACK RIGHT: ", driveTrain.rightDrive.motor2.getVelocity());
        dash.create("BACK LEFT: ", driveTrain.leftDrive.motor2.getVelocity());
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
        driveTrain.setKd(MasqUtils.KD.MOTOR_TELEOP);
    }
    public void initializeAutonomous(){
        driveTrain.setKp(MasqUtils.KP.MOTOR_AUTONOMOUS);
        driveTrain.setKi(MasqUtils.KI.MOTOR_AUTONOMOUS);
        driveTrain.setKd(MasqUtils.KD.MOTOR_AUTONOMOUS);
    }
    public void sleep(double time) {
        try {
            Thread.sleep((long) time * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void sleep() {sleep(MasqUtils.DEFAULT_SLEEP_TIME);}
}
