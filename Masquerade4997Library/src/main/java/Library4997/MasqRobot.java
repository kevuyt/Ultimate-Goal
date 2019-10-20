package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqWrappers.MasqPredicate;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 */
public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    public abstract MasqPIDPackage pidPackage();
    private int timeout = 2;
    public MasqMechanumDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    public DashBoard dash;
    public double velocityMagnitude = 1;
    public double speedMultiplier = 1.4;
    public double turnMultiplier = 1.4;
    private MasqClock timeoutClock = new MasqClock();
    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}
    public void drive(double distance, double speed, Direction direction, double timeOut, int sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining = 0;
        double angularError, prevAngularError = 0, angularIntegral = 0, angularDerivative,
                powerAdjustment, power, leftPower = 0, rightPower = 0, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * pidPackage().getKpDriveEncoder()) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            angularError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
            angularIntegral += angularError*timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (pidPackage().getKpDriveAngular() * power) * angularError + (pidPackage().getKiDriveAngular() * angularIntegral) +
                    (pidPackage().getKdDriveAngular() * angularDerivative);
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
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
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
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 0.5, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeOut, int sleepTime) {
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError = tracker.imu.adjustAngle((double) angle - tracker.getHeading()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower = 0, rightPower = 0, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = ((clicksRemaining / targetClicks) * pidPackage().getKpDriveEncoder()) * direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            timeChange = loopTimer.milliseconds();
            loopTimer.reset();
            loopTimer.reset();
            angularError = tracker.imu.adjustAngle((double)angle - tracker.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            powerAdjustment = (pidPackage().getKpDriveAngular() * power) * angularError + (pidPackage().getKiDriveAngular() * angularIntegral) +
                    (pidPackage().getKdDriveAngular() * angularDerivative);
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
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
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

    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd, boolean left, boolean right) {
        double targetAngle = tracker.imu.adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower;
        double leftPower = 0, rightPower = 0;
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
    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki) {
        turnRelative(angle, direction, timeOut, sleepTime, kp, ki, pidPackage().getKdTurn(), true, true);
    }
    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp) {
        turnRelative(angle, direction, timeOut, sleepTime, kp, pidPackage().getKiTurn());
    }
    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime) {
        turnRelative(angle, direction, timeOut, sleepTime,pidPackage().getKpTurn());
    }
    public void turnRelative(double angle, Direction direction, double timeout) {
        turnRelative(angle, direction, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction direction)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void turnRelative(double angle, Direction direction, boolean left, boolean right)  {
        turnRelative(angle, direction, MasqUtils.DEFAULT_TIMEOUT, MasqUtils.DEFAULT_SLEEP_TIME,
                pidPackage().getKpTurn(), pidPackage().getKiTurn(), pidPackage().getKdTurn(), left, right);
    }

    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        double targetAngle = tracker.imu.adjustAngle((direction.value * angle));
        double acceptableError = 2;
        double currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
        double prevError = 0;
        double integral = 0;
        double derivative;
        double newPower = 0;
        double previousTime = 0;
        timeoutClock.reset();
        while (opModeIsActive() && (tracker.imu.adjustAngle(Math.abs(currentError)) > acceptableError)
                && !timeoutClock.elapsedTime(timeOut, MasqClock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            tChange = tChange / 1e9;
            currentError = tracker.imu.adjustAngle(targetAngle - tracker.getHeading());
            integral += currentError * tChange;
            derivative = (currentError - prevError) / tChange;
            double p = currentError * kp;
            double i = integral * ki;
            double d = derivative * kd;
            newPower = p + i + d;
            if (Math.abs(newPower) >= 1) {newPower /= Math.abs(newPower);}
            driveTrain.setVelocity(-newPower, newPower);
            prevError = currentError;
            dash.create("KP: ", kp);
            dash.create("RIGHT POWER: " ,newPower);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", currentError);
            dash.update();
            previousTime = System.nanoTime();
        }
        driveTrain.setVelocity(0,0);
        sleep(sleepTime);
    }
    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki) {
        turnAbsolute(angle, direction, timeOut, sleepTime, kp, ki, pidPackage().getKdTurn());
    }
    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime, double kp) {
        turnAbsolute(angle, direction, timeOut, sleepTime, kp, pidPackage().getKiTurn());
    }
    public void turnAbsolute(double angle, Direction direction, double timeOut, int sleepTime) {
        turnAbsolute(angle, direction, timeOut, sleepTime, pidPackage().getKpTurn());
    }
    public void turnAbsolute(double angle, Direction direction, double timeout)  {
        turnAbsolute(angle, direction, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle, Direction direction)  {
        turnAbsolute(angle, direction, MasqUtils.DEFAULT_TIMEOUT);
    }

    public void stop(MasqPredicate stopCondtion, double angle, double speed, Direction direction, double timeOut) {
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
            angularError = tracker.imu.adjustAngle(angle - tracker.getHeading());
            angularIntegral = (angularIntegral + angularError) * timeChange;
            angularDerivative = (angularError - prevAngularError) / timeChange;
            prevAngularError = angularError;
            powerAdjustment = (pidPackage().getKpDriveAngular() * power) * angularError + (pidPackage().getKiDriveAngular() * angularIntegral) +
                    (pidPackage().getKdDriveAngular() * angularDerivative);
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
        double l = 10;
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
                    projection.getX() + (l * Math.cos(theta)),
                    projection.getY() + (l * Math.sin(theta)));
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

        driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value * velocityMagnitude);
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
        driveTrain.leftDrive.setKp(pidPackage().getKpMotorTeleOpLeft());
        driveTrain.leftDrive.setKi(pidPackage().getKiMotorTeleOpLeft());
        driveTrain.leftDrive.setKd(pidPackage().getKdMotorTeleOpLeft());
        driveTrain.rightDrive.setKp(pidPackage().getKpMotorTeleOpRight());
        driveTrain.rightDrive.setKi(pidPackage().getKiMotorTeleOpRight());
        driveTrain.rightDrive.setKd(pidPackage().getKdMotorTeleOpRight());
    }
    public void initializeAutonomous(){
        driveTrain.leftDrive.setKp(pidPackage().getKpMotorAutoLeft());
        driveTrain.leftDrive.setKi(pidPackage().getKiMotorAutoLeft());
        driveTrain.leftDrive.setKd(pidPackage().getKdMotorAutoLeft());
        driveTrain.rightDrive.setKp(pidPackage().getKpMotorAutoRight());
        driveTrain.rightDrive.setKi(pidPackage().getKiMotorAutoRight());
        driveTrain.rightDrive.setKd(pidPackage().getKdMotorAutoRight());
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
    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public void setTurnMultiplier(double turnMultiplier) {
        this.turnMultiplier = turnMultiplier;
    }
    public void setVelocityMagnitude(double velocityMagnitude) {
        this.velocityMagnitude = velocityMagnitude;
    }
}
