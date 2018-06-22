package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSensors.MasqPositionTracker;
import Library4997.MasqSensors.MasqTouchSensor;
import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqEncoderModel;
import Library4997.MasqUtilities.MasqUtils;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Robot extends MasqRobot {
    private HardwareMap hardwareMap;
    public MasqTouchSensor touchSensor;
    public MasqPositionTracker positionTracker;
    public MasqMotorSystem yTranslator;
    public MasqMotor leftMotor, rightMotor;
    private MasqClock timeoutClock = new MasqClock();
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        yTranslator = new MasqMotorSystem("y1", "y2", "y", this.hardwareMap);
        touchSensor = new MasqTouchSensor("touchSensor", this.hardwareMap);
        leftMotor = new MasqMotor("leftMotor", this.hardwareMap);
        rightMotor = new MasqMotor("rightMotor", DcMotor.Direction.REVERSE, this.hardwareMap);
        positionTracker = new MasqPositionTracker(this.hardwareMap, leftMotor, MasqEncoderModel.USDIGITAL_E4T,
                rightMotor, MasqEncoderModel.NEVEREST20);
    }


    public void drive(double distance, double speed, Direction direction, double timeOut, int sleepTime) {
        leftMotor.setClosedLoop(true);
        MasqClock timeoutTimer = new MasqClock();
        MasqClock loopTimer = new MasqClock();
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        double targetAngle = positionTracker.getRotation();
        double targetClicks = (int)(distance * MasqUtils.CLICKS_PER_INCH);
        double clicksRemaining;
        double angularError = positionTracker.imu.adjustAngle(targetAngle - positionTracker.getRotation()),
                prevAngularError = angularError, angularIntegral = 0,
                angularDerivative, powerAdjustment, power, leftPower, rightPower, maxPower, timeChange;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(leftMotor.getCurrentPosition()));
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
            leftMotor.setPower(rightPower);
            rightMotor.setPower(leftPower);
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeOut, MasqClock.Resolution.SECONDS) && (clicksRemaining / targetClicks) > 0.05);
        leftMotor.setPower(0);
        sleep(sleepTime);
    }
    public void drive(double distance, double speed, Direction strafe, double timeOut) {
        drive(distance, speed, strafe, timeOut, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, 9);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance) {drive(distance, 0.5);}


    public void turnRelative(double angle, Direction direction, double timeOut, int sleepTime, double kp, double ki, double kd) {
        //positionTracker.startIgnoringRotation();
        leftMotor.setClosedLoop(false);
        rightMotor.setClosedLoop(false);
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
            leftMotor.setPower(-newPower * turnPower);
            rightMotor.setPower(newPower * turnPower);
            prevError = currentError;
            this.angleLeftCover = currentError;
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(sleepTime);
        //positionTracker.endIgnoringRotation();
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

}
