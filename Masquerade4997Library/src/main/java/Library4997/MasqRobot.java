package Library4997;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import Library4997.MasqControlSystems.MasqPID.MasqDrivePIDController;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqResources.MasqUtilsv2;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqWrappers.MasqPredicate;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    private int timeout = 30;
    public BNO055IMU imu;
    public MasqMechanumDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    public DashBoard dash;
    public double speedMultiplier = 1.414;
    public double turnMultiplier = 1.414;
    private MasqClock timeoutClock = new MasqClock();
    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}



    public void strafe (double distance, double angle, double timeout) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double power, timeChange, angularError, angularDerivative, angularIntegral = 0, targetAngle = tracker.getHeading(), prevAngularError = 0, powerAdjustment = 0;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.rightDrive.motor1.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE) * 0.5;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            angularIntegral += angularError*timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = MasqUtils.KP.ANGLE * angularError + (MasqUtils.KI.ANGLE * angularIntegral) +
                    (MasqUtils.KD.ANGLE * angularDerivative);
            driveTrain.setPowerMECH(angle, power, tracker.getHeading(), powerAdjustment);
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && (clicksRemaining / targetClicks) > 0.05);
        driveTrain.stopDriving();
        sleep(MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void strafe(double distance, Direction direction, double timeout) {
        double angle;
        switch (direction) {
            case LEFT:
                angle = -90;
                break;
            case RIGHT:
                angle = 90;
                break;
            case BACKWARD:
                angle = 180;
                break;
            case FORWARD:
            default:
                angle = 0;
                break;
        }
        strafe(distance, angle, timeout);
    }
    public void strafe(double distance, Direction direction) {
        strafe(distance, direction,1);
    }

    public void drive(double distance, double speed, Direction direction, double timeOut, double sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, prevAngularError = 0, angularIntegral = 0, angularDerivative,
                powerAdjustment, power, leftPower = 0, rightPower = 0, maxPower, timeChange;
        MasqDrivePIDController driveController = new MasqDrivePIDController(MasqUtilsv2.driveConstants.kp);
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            clicksRemaining = driveController.getOutput(clicksRemaining/targetClicks);
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            angularIntegral += angularError*timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.ANGLE * angularError) + (MasqUtils.KI.ANGLE * angularIntegral) +
                    (MasqUtils.KD.ANGLE * angularDerivative);
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
            /*dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();*/
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
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 1, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeOut, double sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError = MasqUtils.adjustAngle((double) angle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower = 0, rightPower = 0, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * MasqUtils.KP.DRIVE) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            loopTimer.reset();
            angularError = MasqUtils.adjustAngle((double)angle - tracker.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            powerAdjustment = (MasqUtils.KP.ANGLE * power) * angularError + (MasqUtils.KI.ANGLE * angularIntegral) +
                    (MasqUtils.KD.ANGLE * angularDerivative);
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
            //serializer.writeData(new Object[]{clicksRemaining, power, angularError, angularIntegral, angularDerivative, leftPower, rightPower, powerAdjustment});
           /* dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();*/
            prevAngularError = angularError;
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.1));
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
    public void driveAbsoluteAngle(double distance, int angle, double speed){
        driveAbsoluteAngle(distance, angle, speed, Direction.FORWARD);
    }
    public void driveAbsoluteAngle(double distance, int angle) {
        driveAbsoluteAngle(distance, angle, 0.5);
    }

    public void turnRelative(double angle, Direction direction, double timeOut, double sleepTime, double kp, double ki, double kd, boolean left, boolean right) {
        double targetAngle = MasqUtils.adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double currentError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double leftPower = 0, rightPower = 0;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            currentError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            integral += currentError * tChange;
            derivative = (currentError - prevError) / tChange;
            double p = currentError * kp;
            double i = integral * ki;
            double d = derivative * kd;
            newPower = (p + i + d);
            if (Math.abs(newPower) >= 1) newPower /= Math.abs(newPower);
            if (left) leftPower = -newPower;
            if (right) rightPower = newPower;
            driveTrain.setVelocity(leftPower, rightPower);
            prevError = currentError;
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.create("Power: ", newPower);
            dash.create("Raw Power: ", driveTrain.getPower());
            dash.update();
        }
        driveTrain.setVelocity(0,0);
        sleep(sleepTime);
    }
    public void turnRelative(double angle, Direction direction, double timeOut, double sleepTime, double kp, double ki) {
        turnRelative(angle, direction, timeOut, sleepTime, kp, ki, MasqUtils.KD.TURN, true, true);
    }
    public void turnRelative(double angle, Direction direction, double timeOut, double sleepTime, double kp) {
        turnRelative(angle, direction, timeOut, sleepTime, kp, MasqUtils.KI.TURN);
    }
    public void turnRelative(double angle, Direction direction, double timeOut, double sleepTime) {
        turnRelative(angle, direction, timeOut, sleepTime,MasqUtils.KP.TURN);
    }
    public void turnRelative(double angle, Direction direction, double timeout) {
        turnRelative(angle, direction, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction direction)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void turnRelative(double angle, Direction direction, boolean left, boolean right)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT, MasqUtils.DEFAULT_SLEEP_TIME,
                MasqUtils.KP.TURN, MasqUtils.KI.TURN, MasqUtils.KD.TURN, left, right);
    }

    public void turnAbsolute(double angle,  double timeOut, double sleepTime, double kp, double ki, double kd) {
        double targetAngle = MasqUtils.adjustAngle(angle);
        double acceptableError = 2;
        double currentError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower = 0;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            tChange = tChange / 1e9;
            currentError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            integral += currentError * tChange;
            derivative = (currentError - prevError) / tChange;
            double p = currentError * kp;
            double i = integral * ki;
            double d = derivative * kd;
            newPower = p + i + d;
            if (Math.abs(newPower) >= 1) {newPower /= Math.abs(newPower);}
            driveTrain.setVelocity(-newPower, newPower);
            prevError = currentError;
            /*dash.create("KP: ", kp);
            dash.create("RIGHT POWER: " ,newPower);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();*/
            previousTime = System.nanoTime();
        }
        driveTrain.setVelocity(0,0);
        sleep(sleepTime);
    }
    public void turnAbsolute(double angle, double timeOut, double sleepTime, double kp, double ki) {
        turnAbsolute(angle, timeOut, sleepTime, kp, ki, MasqUtils.KD.TURN);
    }
    public void turnAbsolute(double angle, double timeOut, double sleepTime, double kp) {
        turnAbsolute(angle, timeOut, sleepTime, kp, MasqUtils.KI.TURN);
    }
    public void turnAbsolute(double angle,  double timeOut, double sleepTime) {
        turnAbsolute(angle, timeOut, sleepTime, MasqUtils.KP.TURN);
    }
    public void turnAbsolute(double angle, double timeout)  {
        turnAbsolute(angle, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle)  {
        turnAbsolute(angle, 0.5);
    }

    public void stop(MasqPredicate stopCondtion, double angle, double speed, Direction direction, double timeOut) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double angularError = MasqUtils.adjustAngle(angle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = MasqUtils.adjustAngle(angle - tracker.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (MasqUtils.KP.ANGLE * power) * angularError + (MasqUtils.KI.ANGLE * angularIntegral) +
                    (MasqUtils.KD.ANGLE * angularDerivative);
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
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && stopCondtion.run());
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

    public void xyPath(double x, double y, double heading, double speedDampener, double kp) {
        // https://www.desmos.com/calculator/zbviad1hnz
        double lookAhead = 10;
        MasqPIDController speedController = new MasqPIDController(0.04, 0, 0);
        driveTrain.setTurnKP(kp);
        MasqClock clock = new MasqClock();
        MasqVector target = new MasqVector(x, y);
        MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        MasqVector inital = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        MasqVector pathDisplacment = inital.displacement(target);
        while (!clock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && !current.equal(3, target) && opModeIsActive()) {
            MasqVector untransformedProjection = new MasqVector(
                    current.projectOnTo(pathDisplacment).getX() - inital.getX(),
                    current.projectOnTo(pathDisplacment).getY() - inital.getY()).projectOnTo(pathDisplacment);
            MasqVector projection = new MasqVector(
                    untransformedProjection.getX() + inital.getX(),
                    untransformedProjection.getY() + inital.getY());
            double theta = Math.atan2(pathDisplacment.getY(), pathDisplacment.getX());
            MasqVector lookahead = new MasqVector(
                    projection.getX() + (lookAhead * Math.cos(theta)),
                    projection.getY() + (lookAhead * Math.sin(theta)));
            if (inital.displacement(lookahead).getMagnitude() > pathDisplacment.getMagnitude()) lookahead = new MasqVector(target.getX(), target.getY());
            MasqVector lookaheadDisplacement = current.displacement(lookahead);
            double pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
            double speed = speedController.getOutput(current.displacement(target).getMagnitude());
            driveTrain.setVelocityMECH(pathAngle + tracker.getHeading(), speed * speedDampener, heading);
            current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            tracker.updateSystem();
            dash.create("X: ", tracker.getGlobalX());
            dash.create("Y: ", tracker.getGlobalY());
            dash.create("Angle: ", pathAngle + tracker.getHeading());
            dash.update();
        }
    }
    public void xyPath(double x, double y, double heading, double speedDampener) {
        xyPath(x, y, heading, speedDampener, 0.05);
    }
    public void xyPath(double x, double y, double heading) {
        xyPath(x, y, heading, 1);
    }
    public void xyPath(MasqPoint p, double heading, double speedDampener, double kp) {
        xyPath(p.getX(), p.getY(), heading, speedDampener, kp);
    }
    public void xyPath(MasqPoint p, double heading, double speedDampener) {
        xyPath(p.getX(), p.getY(), heading, speedDampener);
    }
    public void xyPath(MasqPoint p, double heading) {
        xyPath(p.getX(), p.getY(), heading, 1);
    }
    public void xyPath(MasqPoint p) {
        xyPath(p.getX(), p.getY(), p.getH());
    }

    public void gotoXY(double x, double y, double heading, double speedDampener, double kp) {
        MasqClock clock = new MasqClock();
        MasqVector target = new MasqVector(x, y);
        double lookAheadDistance = 10;
        driveTrain.setTurnKP(kp);
        MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
        double targetInches = current.distanceToVector(target);
        while (!clock.elapsedTime(3, MasqClock.Resolution.SECONDS) && !current.equal(2, target) && opModeIsActive()) {
            MasqVector displacement = current.displacement(target);
            double speed = displacement.getMagnitude() / targetInches;
            MasqVector projection = current.projectOnTo(target);
            double theta = Math.atan2(projection.getY(), projection.getX());
            MasqVector lookahead = new MasqVector(projection.getX() + (lookAheadDistance * Math.cos(theta)), projection.getY() + (lookAheadDistance * Math.sin(theta)));
            if (lookahead.getMagnitude() > target.getMagnitude()) lookahead = new MasqVector(target.getX(), target.getY());
            MasqVector lookaheadDisplacement = current.displacement(lookahead);
            double pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
            driveTrain.setVelocityMECH(pathAngle + tracker.getHeading(), speed * speedDampener, heading);
            current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            tracker.updateSystem();
            dash.create("X: ", tracker.getGlobalX());
            dash.create("Y: ", tracker.getGlobalY());
            dash.create("Speed: ", speed * speedDampener);
            dash.update();
        }
    }
    public void gotoXY(double x, double y, double heading, double speedDampener) {
        gotoXY(x, y, heading, speedDampener, 0.05);
    }
    public void gotoXY(double x, double y, double heading) {
        gotoXY(x, y, heading, 1);
    }
    public void gotoXY(MasqPoint p, double heading, double speedDampener, double kp) {
        gotoXY(p.getX(), p.getY(), heading, speedDampener, kp);
    }
    public void gotoXY(MasqPoint p, double heading, double speedDampener) {
        gotoXY(p.getX(), p.getY(), heading, speedDampener);
    }
    public void gotoXY(MasqPoint p, double heading) {
        gotoXY(p.getX(), p.getY(), heading, 1);
    }
    public void gotoXY(MasqPoint p) {
        gotoXY(p.getX(), p.getY(), p.getH());
    }

    public void NFS(MasqController c) {
        float move = c.leftStickY();
        float turn = c.rightStickX();
        double left = move - turn;
        double right = move + turn;
        left *= -1;
        right *= -1;
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
    public void MECH(MasqController c, Direction direction, boolean fieldCentric) {
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
        MECH(c, direction, false);
    }
    public void MECH(MasqController c, boolean disabled) {
        MECH(c, Direction.FORWARD, disabled);
    }
    public void MECH(MasqController c) {
        MECH(c, Direction.FORWARD, false);
    }

    public void initializeTeleop(){
        driveTrain.setKp(MasqUtils.KP.VELOCITY_TELE);
        driveTrain.setKi(MasqUtils.KI.VELOCITY_TELE);
        driveTrain.setKd(MasqUtils.KD.VELOCITY_TELE);
    }
    public void initializeAutonomous() {
        driveTrain.setKp(MasqUtils.KP.VELOCITY_AUTO);
        driveTrain.setKi(MasqUtils.KI.VELOCITY_AUTO);
        driveTrain.setKd(MasqUtils.KD.VELOCITY_AUTO);
    }
    public void setPIDConstants(double kp, double ki, double kd) {
        driveTrain.setKp(kp);
        driveTrain.setKi(ki);
        driveTrain.setKd(kd);
    }

    public void sleep(double timeSeconds) {
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {sleep(MasqUtils.DEFAULT_SLEEP_TIME);}
    public WebcamName getWebCameName (HardwareMap hardwareMap, String name) {
        return hardwareMap.get(WebcamName.class, name);
    }

    public void setMultipliers(double multiplier) {
        speedMultiplier = multiplier;
        turnMultiplier = multiplier;
    }
    public void setTurnMultiplier(double turnMultiplier) {
        this.turnMultiplier = turnMultiplier;
    }
    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }
}
