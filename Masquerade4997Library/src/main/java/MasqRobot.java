import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import MasqueradeLibrary.MasqMath.MasqPIDController;
import MasqueradeLibrary.MasqMath.MasqVector;
import MasqueradeLibrary.MasqMath.MasqWayPoint;
import MasqueradeLibrary.MasqMotion.MasqDriveTrain;
import MasqueradeLibrary.MasqPositionTracker;
import MasqueradeLibrary.MasqResources.DashBoard;
import MasqueradeLibrary.MasqResources.MasqClock;
import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqMath.MasqWayPoint.PointMode;
import static MasqueradeLibrary.MasqMath.MasqWayPoint.PointMode.*;
import static MasqueradeLibrary.MasqResources.MasqClock.Resolution.SECONDS;
import static MasqueradeLibrary.MasqResources.MasqUtils.*;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.*;
import static java.util.Arrays.asList;

/**
 * Created by Keval Kataria on 3/15/2021
 * TODO
 *  Rewrite Drive method to use tank odometry
 */

public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    public abstract void init(HardwareMap hardwareMap, OpMode opmode);

    public MasqDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    private MasqClock timeoutClock = new MasqClock("Timeout Clock");
    protected DashBoard dash;

    public enum OpMode {
        AUTO, TELEOP
    }

    public void drive(double distance, double timeout) {
        double targetAngle = tracker.getHeading();
        double targetClicks = distance * driveTrain.model.CPR;
        double clicksRemaining, angularError, powerAdjustment, power, leftPower, rightPower, maxPower;

        driveTrain.resetEncoders();
        timeoutClock.reset();
        do {
            clicksRemaining = targetClicks - abs(driveTrain.getCurrentPosition());
            power = driveController.getOutput(clicksRemaining);
            power = clip(power, -1, 1);
            angularError = adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = clip(powerAdjustment, -1, 1);
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;

            maxPower = max(abs(leftPower), abs(rightPower));
            if (maxPower > 1) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            tracker.updateSystem();
            driveTrain.setPower(leftPower, rightPower);

            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && timeoutClock.hasNotPassed(timeout, SECONDS) && (abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setPower(0);
    }
    public void drive(double distance) {drive(distance, DEFAULT_TIMEOUT);}

    public void turnAbsolute(double angle, double timeout) {
        double error, power;

        timeoutClock.reset();
        do {
            error = adjustAngle(angle - tracker.getHeading());
            power = clip(turnController.getOutput(error), -1, 1);

            driveTrain.setPower(power, -power);
            tracker.updateSystem();

            dash.create("KP: ", turnController.getConstants()[0]);
            dash.create("Power: " , power);
            dash.create("TargetAngle: ", angle);
            dash.create("Heading: ", tracker.getHeading());
            dash.create("AngleLeftToCover: ", error);
            dash.update();
        } while (opModeIsActive() && (adjustAngle(abs(error)) > 1) && timeoutClock.hasNotPassed(timeout, SECONDS));
        driveTrain.setPower(0);
    }
    public void turnAbsolute(double angle) {turnAbsolute(angle, DEFAULT_TIMEOUT);}

    public void turnRelative(double angle, double timeout) {turnAbsolute(tracker.getHeading() + angle, timeout);}
    public void turnRelative(double angle) {turnAbsolute(tracker.getHeading() + angle);}

    public void xyPath(double timeout, MasqWayPoint... points) {
        List<MasqWayPoint> pointsWithRobot = new ArrayList<>(asList(points));
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
            MasqWayPoint target = pointsWithRobot.get(index);
            MasqVector current = new MasqVector("Current", tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector("Initial", pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            double pathAngle;
            pointTimeout.reset();
            while (pointTimeout.hasNotPassed(pointsWithRobot.get(index).getTimeout(), SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target.getPoint()) && opModeIsActive() && speed > 0.1) {
                double heading = toRadians(tracker.getHeading());
                MasqVector headingUnitVector = new MasqVector("Heading Unit Vector", sin(heading), cos(heading));
                MasqVector lookahead = getLookAhead(initial, current, target.getPoint(), lookAheadDistance);
                MasqVector pathDisplacement = initial.displacement(target.getPoint());
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = target.getPoint();
                    else break;
                }
                MasqVector lookaheadDisplacement = current.displacement(lookahead);
                speed = speedController.getOutput(current.displacement(target.getPoint()).getMagnitude());
                speed = scaleNumber(speed, target.getMinVelocity(), target.getMaxVelocity());

                PointMode mode = target.getSwitchMode();
                boolean mechMode =(current.equal(target.getModeSwitchRadius(), target.getPoint()) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    double turnPower = angleController.getOutput(adjustAngle(target.getH() - tracker.getHeading()));
                    pathAngle = 90 - toDegrees(atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setPowerMECH(pathAngle - tracker.getHeading(), speed, turnPower);
                }
                else {
                    pathAngle = adjustAngle(headingUnitVector.angleTo(lookaheadDisplacement));
                    double powerAdjustment = angleController.getOutput(pathAngle);
                    double leftPower = speed + powerAdjustment;
                    double rightPower = speed - powerAdjustment;

                    int direction = 1;
                    if(abs(pathAngle) > 100) direction = -1;

                    driveTrain.setPower(direction * leftPower, direction * rightPower);
                }

                tracker.updateSystem();

                dash.create(tracker);
                dash.create("Distance Left", target.getPoint().displacement(current).getMagnitude());
                dash.create("Path Angle: ", pathAngle);
                dash.update();

                current.setX(tracker.getGlobalX());
                current.setY(tracker.getGlobalY());
            }
            pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        driveTrain.setPower(0);
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
        driveTrain.rightDrive.setPower(c.right_stick_y);
        driveTrain.leftDrive.setPower(c.left_stick_y);
    }

    public void MECH(Gamepad c) {
        double x = c.left_stick_x;
        double y = -c.left_stick_y;
        double xR = c.right_stick_x;
        double angle = atan2(x, y);

        driveTrain.setPowerMECH(angle,hypot(x, y),xR);
    }
    public void MECH() {MECH(getLinearOpMode().getDefaultController());}

    public MasqWayPoint getCurrentWayPoint() {
        return new MasqWayPoint().setPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading()).setName("Inital WayPoint");
    }
}