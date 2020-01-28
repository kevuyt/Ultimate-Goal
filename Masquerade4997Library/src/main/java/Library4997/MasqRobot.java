package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPointLegacy;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqWrappers.MasqPredicate;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.SWITCH;
import static Library4997.MasqResources.MasqUtils.DEFAULT_TIMEOUT;
import static Library4997.MasqResources.MasqUtils.angleController;
import static Library4997.MasqResources.MasqUtils.driveController;
import static Library4997.MasqResources.MasqUtils.scaleNumber;
import static Library4997.MasqResources.MasqUtils.tolerance;
import static Library4997.MasqResources.MasqUtils.turnController;
import static Library4997.MasqResources.MasqUtils.velocityAutoController;
import static Library4997.MasqResources.MasqUtils.velocityTeleController;
import static Library4997.MasqResources.MasqUtils.xyAngleController;
import static Library4997.MasqResources.MasqUtils.xySpeedController;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */

/*
TODO:
    Path Control
    Unit Tests for all major functions
    State Machine support
 */
public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    public abstract void init(HardwareMap hardwareMap) throws InterruptedException;

    public MasqMechanumDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    public DashBoard dash;
    private MasqClock timeoutClock = new MasqClock();

    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}

    public void strafe(double distance, double angle, double timeout, double speed) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double power, angularError, targetAngle = tracker.getHeading(), powerAdjustment;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPositionPositive()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MasqUtils.adjustAngle(angularError));
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            driveTrain.setVelocityMECH(angle, power, tracker.getHeading(), powerAdjustment);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.stopDriving();
        MasqUtils.sleep(MasqUtils.DEFAULT_SLEEP_TIME);
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
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MasqUtils.adjustAngle(angularError));
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            leftPower = (direction.value * power) - powerAdjustment;
            rightPower = (direction.value * power) + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.stopDriving();
        MasqUtils.sleep(sleepTime);
    }
    public void drive(double distance, double speed, Direction strafe, double timeout) {
        drive(distance, speed, strafe, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 1, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeout, double sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            angularError = MasqUtils.adjustAngle((double)angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            leftPower*=direction.value;
            rightPower*=direction.value;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            //serializer.writeData(new Object[]{clicksRemaining, power, angularError, angularIntegral, angularDerivative, leftPower, rightPower, powerAdjustment});
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.01));
        //serializer.close();
        driveTrain.stopDriving();
        MasqUtils.sleep(sleepTime);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe, double timeout) {
        driveAbsoluteAngle(distance, angle, speed, strafe, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe) {
        driveAbsoluteAngle(distance, angle, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed){
        driveAbsoluteAngle(distance, angle, speed, Direction.FORWARD);
    }
    public void driveAbsoluteAngle(double distance, int angle) {
        driveAbsoluteAngle(distance, angle, 0.5);
    }

    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp, double ki, double kd, boolean left, boolean right) {
        double targetAngle = MasqUtils.adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double error = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
        double power;
        double leftPower = 0, rightPower = 0;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            error = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
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
        MasqUtils.sleep(sleepTime);
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
        turnRelative(angle, direction, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction direction)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void turnRelative(double angle, Direction direction, boolean left, boolean right)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT, MasqUtils.DEFAULT_SLEEP_TIME,
                turnController.getConstants()[0], turnController.getConstants()[1], turnController.getConstants()[2], left, right);
    }

    public void turnAbsolute(double angle,  double timeout, double sleepTime, double kp, double ki, double kd) {
        //double targetAngle = MasqUtils.adjustAngle(angle);
        double acceptableError = 2;
        double error = MasqUtils.adjustAngle(angle - tracker.getHeading());
        double power;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            error = MasqUtils.adjustAngle(angle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
            driveTrain.setVelocity(-power, power);
            dash.create("KP: ", kp);
            dash.create("RIGHT POWER: " ,power);
            dash.create("TargetAngle", angle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.update();
        }
        driveTrain.setVelocity(0,0);
        MasqUtils.sleep(sleepTime);
    }
    public void turnAbsolute(double angle, double timeout, double sleepTime, double kp, double ki) {
        turnAbsolute(angle, timeout, sleepTime, kp, ki, turnController.getKd());
    }
    public void turnAbsolute(double angle, double timeout, double sleepTime, double kp) {
        turnAbsolute(angle, timeout, sleepTime, kp, turnController.getKi());
    }
    public void turnAbsolute(double angle,  double timeout, double sleepTime) {
        turnAbsolute(angle, timeout, sleepTime, turnController.getKp());
    }
    public void turnAbsolute(double angle, double timeout)  {
        turnAbsolute(angle, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle)  {
        turnAbsolute(angle, DEFAULT_TIMEOUT);
    }

    public void stop(MasqPredicate stopCondtion, double angle, double speed, Direction direction, double timeout) {
        MasqClock timeoutTimer = new MasqClock();
        driveTrain.resetEncoders();
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MasqUtils.adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("Angle Error", angularError);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && stopCondtion.run());
        driveTrain.stopDriving();
    }
    public void stop(MasqPredicate stopCondition, double angle, double speed, Direction direction) {
        stop(stopCondition, angle, speed, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void stop(MasqPredicate sensor, double angle, double power) {
        stop(sensor, angle, power, Direction.FORWARD);
    }
    public void stop(MasqPredicate stopCondition, double angle) {
        stop(stopCondition, angle, 0.5);
    }
    public void stop(MasqPredicate sensor){
        stop(sensor, tracker.getHeading());
    }
    public void stop(MasqPredicate stopCondition, int timeout) {
        stop(stopCondition, tracker.getHeading(), 0.5, Direction.FORWARD, timeout);
    }



    public void gotoXY(double x, double y, double heading, double timeout, double radius, double speedDampener) {
        // https://www.desmos.com/calculator/zbviad1hnz
        double lookAhead = 10;
        MasqMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MasqMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MasqMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        MasqClock clock = new MasqClock();
        MasqVector target = new MasqVector(x, y);
        MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        MasqVector initial = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        MasqVector pathDisplacement = initial.displacement(target);
        double speed = 1;
        while (!clock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && !current.equal(radius, target) && opModeIsActive() && speed > 0.1) {
            MasqVector untransformedProjection = new MasqVector(
                    current.projectOnTo(pathDisplacement).getX() - initial.getX(),
                    current.projectOnTo(pathDisplacement).getY() - initial.getY()).projectOnTo(pathDisplacement);
            MasqVector projection = new MasqVector(
                    untransformedProjection.getX() + initial.getX(),
                    untransformedProjection.getY() + initial.getY());
            double theta = Math.atan2(pathDisplacement.getY(), pathDisplacement.getX());
            MasqVector lookahead = new MasqVector(
                    projection.getX() + (lookAhead * Math.cos(theta)),
                    projection.getY() + (lookAhead * Math.sin(theta)));
            if (initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude()) lookahead = new MasqVector(target.getX(), target.getY());
            MasqVector lookaheadDisplacement = current.displacement(lookahead);
            double pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
            speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
            driveTrain.setVelocityMECH(pathAngle + tracker.getHeading(), speed * speedDampener, heading);
            current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            tracker.updateSystem();
            dash.create("X: ", tracker.getGlobalX());
            dash.create("Y: ", tracker.getGlobalY());
            dash.create("LEFT POWER: ", driveTrain.leftDrive.getPower());
            dash.create("RIGHT POWER: ", driveTrain.rightDrive.getPower());
            dash.create("Angle: ", pathAngle + tracker.getHeading());
            dash.update();
        }
        driveTrain.setPower(0);
    }
    public void gotoXY(MasqPoint p, double timeout, double radius, double speedDampener) {
        gotoXY(p.getX(), p.getY(), p.getH(), timeout,radius, speedDampener);
    }
    public void gotoXY(MasqPoint p, double timeout,double radius) {
        gotoXY(p, timeout, radius, 0.7);
    }
    public void gotoXY(MasqPoint p, double timeout) {
        gotoXY(p, timeout, 0.5);
    }
    public void gotoXY(MasqPoint p) {
        gotoXY(p,DEFAULT_TIMEOUT);
    }

    public void xyPathLegacy(double lookAhead, MasqWayPointLegacy... points) {
        List<MasqWayPointLegacy> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0,getCurrentWayPointLegacy());
        MasqMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MasqMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MasqMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        int index = 1;
        while (index < pointsWithRobot.size() ) {
            MasqVector target = new MasqVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector(pointsWithRobot.get(index-1).getX(), pointsWithRobot.get(index-1).getY());
            MasqVector pathDisplacement = initial.displacement(target);
            double speed = 1;
            while (!current.equal(pointsWithRobot.get(index).getRadius(), target) && opModeIsActive() && speed > 0.1) {
                MasqVector untransformedProjection = new MasqVector(
                        current.projectOnTo(pathDisplacement).getX() - initial.getX(),
                        current.projectOnTo(pathDisplacement).getY() - initial.getY()).projectOnTo(pathDisplacement);
                MasqVector projection = new MasqVector(
                        untransformedProjection.getX() + initial.getX(),
                        untransformedProjection.getY() + initial.getY());
                double theta = Math.atan2(pathDisplacement.getY(), pathDisplacement.getX());
                MasqVector lookahead = new MasqVector(
                        projection.getX() + (lookAhead * Math.cos(theta)),
                        projection.getY() + (lookAhead * Math.sin(theta)));
                if (initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude())
                    lookahead = new MasqVector(target.getX(), target.getY());
                MasqVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                speed = scaleNumber(speed, pointsWithRobot.get(index).getVelocity(), 1);
                driveTrain.setVelocityMECH(pathAngle + tracker.getHeading(), speed * 0.7, pointsWithRobot.get(index).getH());
                current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
                tracker.updateSystem();
                dash.create("X: ", tracker.getGlobalX());
                dash.create("Y: ", tracker.getGlobalY());
                dash.create("LEFT POWER: ", driveTrain.leftDrive.getPower());
                dash.create("RIGHT POWER: ", driveTrain.rightDrive.getPower());
                dash.create("Angle: ", pathAngle + tracker.getHeading());
                dash.update();
            }
            index++;
        }
        driveTrain.stopDriving();
    }

    public void xyPathLegacy(MasqWayPointLegacy... points) {
        xyPathLegacy(10, points);
    }


    public void xyPath(double timeout, MasqWayPoint... points) {
        MasqMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MasqMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MasqMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        List<MasqWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MasqPIDController travelAngleController = new MasqPIDController(0.01, 0, 0);
        int index = 1;
        MasqClock pointTimeout = new MasqClock();
        timeoutClock.reset();
        while (!timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            travelAngleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MasqMechanumDriveTrain.angleCorrectionController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MasqVector target = new MasqVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            pointTimeout.reset();
            while (!pointTimeout.elapsedTime(pointsWithRobot.get(index).getTimeout(), MasqClock.Resolution.SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target) && opModeIsActive() && speed > 0.1) {
                double heading = Math.toRadians(-tracker.getHeading());
                MasqVector headingUnitVector = new MasqVector(Math.sin(heading), Math.cos(heading));
                MasqVector lookahead = MasqUtils.getLookAhead(initial, current, target, lookAheadDistance);
                MasqVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = new MasqVector(target.getX(), target.getY());
                    else break;
                }
                MasqVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = MasqUtils.adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                double maxVelocity = pointsWithRobot.get(index).getMaxVelocity();
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), maxVelocity);
                double powerAdjustment = travelAngleController.getOutput(pathAngle);
                double leftPower = speed + powerAdjustment;
                double rightPower = speed - powerAdjustment;

                MasqWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode = approachingFinalPos ||
                        (current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            -pointsWithRobot.get(index).getH()
                    );
                }
                else driveTrain.setVelocity(leftPower, rightPower);

                dash.create("Left Power: ", leftPower);
                dash.create("Right Power: " , rightPower);
                dash.create("Power Adjustment: ", powerAdjustment);
                dash.create("pathAngle: ", pathAngle);

                tracker.updateSystem();
                current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
                int i = 0;
                for (MasqWayPoint point : pointsWithRobot) {
                    dash.create(point.setName("Index: " + i));
                    i++;
                }
                dash.update();
            }
            index++;
        }
        driveTrain.stopDriving();
    }

    public void NFS(MasqController c) {
        float move = -c.leftStickY();
        float turn = c.rightStickX() * 0.7f;
        double left = move + turn;
        double right = move - turn;
        double max = MasqUtils.max(left, right);
        if(max > 1.0) {
            left /= max;
            right /= max;
        }
        driveTrain.setPower(left, right);
    }

    public void TANK(MasqController c) {
        double left = -c.leftStickY();
        double right = -c.rightStickY();
        double leftRate = driveTrain.leftDrive.getVelocity();
        double rightRate = driveTrain.rightDrive.getVelocity();
        double maxRate = MasqUtils.max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setPower(right);
        driveTrain.leftDrive.setPower(left);
    }

    public void MECH(MasqController c, Direction direction, boolean fieldCentric, double speedMultiplier, double turnMultiplier) {
        int disable = 0;
        if (fieldCentric) disable = 1;

        double angle;

        double x = -c.leftStickY();
        double y = c.leftStickX();
        double xR = -c.rightStickX();

        angle = Math.atan2(y, x) + (Math.toRadians(tracker.getHeading()) * disable);
        double adjustedAngle = angle + Math.PI/4;

        double speedMagnitude = Math.hypot(x, y);

        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) - xR * turnMultiplier * direction.value;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) - xR  * turnMultiplier * direction.value;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;

        double max = MasqUtils.max(Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightFront), Math.abs(rightBack));
        if (max > 1) {
            leftFront /= Math.abs(max);
            leftBack /= Math.abs(max);
            rightFront /= Math.abs(max);
            rightBack /= Math.abs(max);
        }

        driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value);
        driveTrain.leftDrive.motor2.setVelocity(leftBack * direction.value);
        driveTrain.rightDrive.motor1.setVelocity(rightFront * direction.value);
        driveTrain.rightDrive.motor2.setVelocity(rightBack * direction.value);
    }
    public void MECH(MasqController c, Direction direction) {
        MECH(c, direction, false, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c, boolean disabled) {
        MECH(c, Direction.FORWARD, disabled, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c) {
        MECH(c, Direction.FORWARD, false, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c, double speedMutliplier, double turnMultiplier) {
        MECH(c, Direction.FORWARD, false, speedMutliplier, turnMultiplier);
    }

    public void initializeTeleop(){
        driveTrain.setKp(velocityTeleController.getKp());
        driveTrain.setKi(velocityTeleController.getKi());
        driveTrain.setKd(velocityTeleController.getKd());
    }
    public void initializeAutonomous() {
        driveTrain.setKp(velocityAutoController.getKp());
        driveTrain.setKi(velocityAutoController.getKi());
        driveTrain.setKd(velocityAutoController.getKd());
    }
    public MasqWayPoint getCurrentWayPoint() {
        return new MasqWayPoint().setPoint(new MasqPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading())).setName("Inital WayPoint");
    }
    public MasqWayPointLegacy getCurrentWayPointLegacy() {
        return new MasqWayPointLegacy(new MasqPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading()));
    }
}