package Library4997;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.*;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMath.*;
import Library4997.MasqResources.DashBoard;
import Library4997.MasqResources.Direction;
import Library4997.MasqMath.MasqVector;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqPositionTracker.*;

import static Library4997.MasqResources.Direction.FORWARD;
import static Library4997.MasqUtils.*;
import static Library4997.MasqSensors.MasqClock.Resolution.SECONDS;
import static Library4997.MasqMath.MasqWayPoint.PointMode.*;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.*;

/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */

public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap) throws InterruptedException;
    public abstract void init(HardwareMap hardwareMap, OpMode opmode) throws InterruptedException;

    public MasqMechanumDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    private MasqClock timeoutClock = new MasqClock();
    protected DashBoard dash;

    public enum OpMode {
        AUTO, TELEOP
    }

    public void strafe(double distance, double angle, double timeout, double speed) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double power, angularError, targetAngle = tracker.getHeading(), powerAdjustment;
        do {
            clicksRemaining = (int) (targetClicks - abs(driveTrain.getCurrentPositionPositive()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = clip(power,-1,1);
            angularError = adjustAngle(targetAngle - tracker.getHeading());
            driveTrain.setVelocityMECH(angle, power, tracker.getHeading());
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && timeoutTimer.hasNotPassed(timeout, SECONDS) && (abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        sleep(DEFAULT_SLEEP_TIME);
    }
    public void strafe(double distance, double angle, double timeout) {
        strafe(distance, angle, timeout, 0.7);
    }
    public void strafe (double distance, double angle) {
        strafe(distance, angle, 1);
    }

    public void drive(double distance, double speed, Direction direction, double timeout, double sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = (int) (targetClicks - abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = clip(power, -1.0, +1.0);
            angularError = adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(adjustAngle(angularError));
            powerAdjustment = clip(powerAdjustment, -1.0, +1.0);
            leftPower = (direction.value * power) - powerAdjustment;
            rightPower = (direction.value * power) + powerAdjustment;
            maxPower = max(abs(leftPower), abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            tracker.updateSystem();
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && timeoutTimer.hasNotPassed(timeout, SECONDS) && (abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        sleep(sleepTime);
    }
    public void drive(double distance, double speed, Direction direction, double timeout) {
        drive(distance, speed, direction, timeout, DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction direction) {
        drive(distance, speed, direction, DEFAULT_TIMEOUT);
    }
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 1, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeout, double sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = (int) (targetClicks - abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            angularError = adjustAngle((double)angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            leftPower*=direction.value;
            rightPower*=direction.value;
            maxPower = max(abs(leftPower), abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            tracker.updateSystem();
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && timeoutTimer.hasNotPassed(timeout, SECONDS) && ((clicksRemaining / targetClicks) > 0.01));
        driveTrain.setVelocity(0);
        sleep(sleepTime);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe, double timeout) {
        driveAbsoluteAngle(distance, angle, speed, strafe, timeout, DEFAULT_SLEEP_TIME);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe) {
        driveAbsoluteAngle(distance, angle, speed, strafe, DEFAULT_TIMEOUT);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed){
        driveAbsoluteAngle(distance, angle, speed, FORWARD);
    }
    public void driveAbsoluteAngle(double distance, int angle) {
        driveAbsoluteAngle(distance, angle, 0.5);
    }

    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp, double ki, double kd, boolean left, boolean right) {
        double targetAngle = adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double error = adjustAngle(targetAngle - tracker.getHeading());
        double power;
        double leftPower = 0, rightPower = 0;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (adjustAngle(abs(error)) > acceptableError)
                && timeoutClock.hasNotPassed(timeout, SECONDS)) {
            error = adjustAngle(targetAngle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (abs(power) >= 1) power /= abs(power);
            if (left) leftPower = power;
            if (right) rightPower = -power;
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.create("Power: ", power);
            dash.create("Raw Power: ", driveTrain.getPower());
            dash.update();
        }
        driveTrain.setVelocity(0,0);
        sleep(sleepTime);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp, double ki) {
        turnRelative(angle, direction, timeout, sleepTime, kp, ki, turnController.getConstants()[2], true, true);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp) {
        turnRelative(angle, direction, timeout, sleepTime, kp, turnController.getConstants()[1]);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime) {
        turnRelative(angle, direction, timeout, sleepTime, turnController.getConstants()[0]);
    }
    public void turnRelative(double angle, Direction direction, double timeout) {
        turnRelative(angle, direction, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction direction)  {
        turnRelative(angle, direction, DEFAULT_TIMEOUT);
    }
    public void turnRelative(double angle, Direction direction, boolean left, boolean right)  {
        turnRelative(angle, direction, DEFAULT_TIMEOUT, DEFAULT_SLEEP_TIME,
                turnController.getConstants()[0], turnController.getConstants()[1], turnController.getConstants()[2], left, right);
    }

    public void turnAbsolute(double angle,  double timeout, double acceptableError, double sleepTime, double kp, double ki, double kd) {
        double error = adjustAngle(angle - tracker.getHeading());
        double power;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (adjustAngle(abs(error)) > acceptableError)
                && timeoutClock.hasNotPassed(timeout, SECONDS)) {
            error = adjustAngle(angle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (abs(power) >= 1) power /= abs(power);
            driveTrain.setVelocity(-power, power);
            tracker.updateSystem();
            dash.create("KP: ", kp);
            dash.create("RIGHT POWER: " ,power);
            dash.create("TargetAngle", angle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.update();
        }
        driveTrain.setVelocity(0,0);
    }
    public void turnAbsolute(double angle, double timeout, double acceptableError, double sleepTime,  double kp, double ki) {
        turnAbsolute(angle, timeout, acceptableError, sleepTime,  kp, ki, turnController.getKd());
    }
    public void turnAbsolute(double angle, double timeout, double acceptableError, double sleepTime, double kp) {
        turnAbsolute(angle, timeout, acceptableError, sleepTime, kp, turnController.getKi());
    }
    public void turnAbsolute(double angle,  double timeout, double acceptableError, double sleepTime) {
        turnAbsolute(angle, timeout, acceptableError, sleepTime,turnController.getKp());
    }
    public void turnAbsolute(double angle, double timeout, double acceptableError) {
        turnAbsolute(angle, timeout, acceptableError, DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle, double timeout)  {
        turnAbsolute(angle, timeout, 1);
    }
    public void turnAbsolute(double angle) {turnAbsolute(angle, DEFAULT_TIMEOUT);}

    public void stopWhen(boolean stopCondition, double angle, double speed, Direction direction, double timeout) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            power = direction.value * speed;
            power = clip(power, -1.0, +1.0);
            angularError = adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = max(abs(leftPower), abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            tracker.updateSystem();
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("Angle Error", angularError);
            dash.update();
        } while (opModeIsActive() && timeoutTimer.hasNotPassed(timeout, SECONDS) && !stopCondition);
        driveTrain.setVelocity(0);
    }
    public void stopWhen(boolean stopCondition, double angle, double speed, Direction direction) {
        stopWhen(stopCondition, angle, speed, direction, DEFAULT_TIMEOUT);
    }
    public void stopWhen(boolean sensor, double angle, double power) {
        stopWhen(sensor, angle, power, FORWARD);
    }
    public void stopWhen(boolean stopCondition, double angle) {
        stopWhen(stopCondition, angle, 0.5);
    }
    public void stopWhen(boolean sensor){
        stopWhen(sensor, tracker.getHeading());
    }
    public void stopWhen(boolean stopCondition, int timeout) {
        stopWhen(stopCondition, tracker.getHeading(), 0.5, FORWARD, timeout);
    }

    public void xyPath(double timeout, MasqWayPoint... points) {
        List<MasqWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MasqPIDController speedController = new MasqPIDController();
        int index = 1;
        MasqClock pointTimeout = new MasqClock();
        timeoutClock.reset();
        while (timeoutClock.hasNotPassed(timeout, SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            angleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            speedController.setKp(pointsWithRobot.get(index).getDriveCorrectionSpeed());
            MasqVector target = new MasqVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            double heading = toRadians(-tracker.getHeading());
            pointTimeout.reset();
            while (pointTimeout.hasNotPassed(pointsWithRobot.get(index).getTimeout(), SECONDS) &&
                    !(current.equal(pointsWithRobot.get(index).getTargetRadius(), target) &&
                    (abs(toDegrees(heading - pointsWithRobot.get(index).getH())) < pointsWithRobot.get(index).getAcceptableError())) &&
                    opModeIsActive() && speed > 0.1) {
                heading = toRadians(-tracker.getHeading());
                MasqVector headingUnitVector = new MasqVector(Math.sin(heading), Math.cos(heading));
                MasqVector lookahead = getLookAhead(initial, current, target, lookAheadDistance);
                MasqVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = target;
                    else break;
                }
                MasqVector  lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = speedController.getOutput(current.displacement(target).getMagnitude());
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), pointsWithRobot.get(index).getMaxVelocity());
                if(current.equal(pointsWithRobot.get(index).getTargetRadius(), target)) speed /= 10;
                double powerAdjustment = angleController.getOutput(pathAngle);
                double leftPower = speed + powerAdjustment;
                double rightPower = speed - powerAdjustment;

                MasqWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode =(current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - toDegrees(atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            -pointsWithRobot.get(index).getH()
                    );
                }
                else {
                    int direction = 1;
                    if(abs(pathAngle) > 100) direction = -1;
                    driveTrain.setVelocity(direction * leftPower, direction * rightPower);
                }

                tracker.updateSystem();

                dash.create("X: "+ tracker.getGlobalX());
                dash.create("Y: "+ tracker.getGlobalY());
                dash.create("Look Ahead Displacement X: ", lookaheadDisplacement.getX());
                dash.create("Look Ahead Displacement Y: ", lookaheadDisplacement.getY());
                dash.create("Distance Left", target.displacement(current).getMagnitude());
                dash.create("Path Angle: ", pathAngle);
                dash.update();

                current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            }
            pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        stop();
    }
    public void xyPath(MasqWayPoint... points) {
        double timeout = 0;
        for(MasqWayPoint point : points) timeout += point.getTimeout();
        xyPath(timeout, points);
    }

    public void NFS(Gamepad c) {
        float move = -c.left_stick_y;
        float turn = c.right_stick_x * 0.7f;
        double left = move + turn;
        double right = move - turn;
        double max = max(left, right);
        if(max > 1.0) {
            left /= max;
            right /= max;
        }
        driveTrain.setPower(left, right);
    }

    public void TANK(Gamepad c) {
        driveTrain.rightDrive.setVelocity(c.right_stick_y);
        driveTrain.leftDrive.setVelocity(c.left_stick_y);
    }

    public void MECH(Gamepad c, Direction direction, boolean fieldCentric, double speedMultiplier, double turnMultiplier, boolean power) {
        int disable = 0;
        if (fieldCentric) disable = 1;

        double x = c.left_stick_x;
        double y = -c.left_stick_y;
        double xR = c.right_stick_x;


        double angle = Math.atan2(x, y) + (toRadians(tracker.getHeading()) * disable);
        double adjustedAngle = angle + Math.PI/4;

        double speedMagnitude = Math.hypot(x, y) * speedMultiplier * direction.value;
        double turnMagnitude = xR * turnMultiplier;

        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude) + turnMagnitude;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude) + turnMagnitude;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude) - turnMagnitude;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude) - turnMagnitude;

        double max = max(abs(leftFront), abs(leftBack), abs(rightFront), abs(rightBack));
        if(max > 1) {
            leftFront /= abs(max);
            leftBack /= abs(max);
            rightFront /= abs(max);
            rightBack /= abs(max);
        }

        if (power) driveTrain.setPowers(leftFront, leftBack, rightFront, rightBack);
        else driveTrain.setVelocities(leftFront, leftBack, rightFront, rightBack);
    }
    public void MECH(Gamepad c, double speedMutliplier, double turnMultiplier) {
        MECH(c, FORWARD, false, speedMutliplier, turnMultiplier, false);
    }
    public void MECH(Gamepad c, boolean disabled) {
        MECH(c, FORWARD, disabled, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, false);
    }
    public void MECH(Gamepad c, boolean fieldCentric, boolean power) {
        MECH(c, FORWARD, fieldCentric, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, power);
    }
    public void MECH(Gamepad c) {
        MECH(c, FORWARD, false, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, false);
    }
    public void MECH(boolean fieldCentric) {
        MECH(getLinearOpMode().getDefaultController(), fieldCentric);
    }
    public void MECH() {MECH(getLinearOpMode().getDefaultController());}

    public MasqWayPoint getCurrentWayPoint() {
        return new MasqWayPoint().setPoint(new MasqPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading())).setName("Inital WayPoint");
    }

    public void stop(double time) {
        MasqClock clock = new MasqClock();
        while(clock.hasNotPassed(time, SECONDS) && opModeIsActive()) {
            driveTrain.setVelocity(0);
        }
        driveTrain.setPower(0);
    }
    public void stop() {
        MasqClock clock = new MasqClock();
        while(clock.hasNotPassed(1, SECONDS) && opModeIsActive()) {
            driveTrain.setVelocity(0);
        }
        driveTrain.setPower(0);
    }
}