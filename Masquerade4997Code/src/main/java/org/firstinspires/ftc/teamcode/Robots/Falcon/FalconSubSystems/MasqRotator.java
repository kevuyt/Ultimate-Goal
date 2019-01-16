package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.atomic.AtomicBoolean;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem, Runnable {
    public MasqMotor rotator;
    private double targetPosition;
    private AtomicBoolean close = new AtomicBoolean(false);
    private AtomicBoolean movementAllowed = new AtomicBoolean(true);
    private double basePower = 0.9;
    private double kp_kp = 5;
    private double baseDownPower = 0.9;
    private double rotatorPower = basePower;
    private double downPower = -0.1;
    private double liftPosition = 0;
    private double kp = 0.01, ki, kd;
    public MasqPIDController output = new MasqPIDController(0.03, 0.0, 0.00);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotor("rotator", MasqMotorModel.NEVEREST40, DcMotor.Direction.FORWARD, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        kp = (1e-6 * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * kp_kp * -liftPosition) + basePower;
        downPower = (0.001 * kp_kp * Math.abs(rotator.getCurrentPosition())) + baseDownPower;
        if (controller.leftTriggerPressed()) rotator.setPower(1);
        else if (controller.rightTriggerPressed()) rotator.setPower(-1);

        if (controller.rightTrigger() > 0 || controller.leftTrigger() > 0) targetPosition = rotator.getCurrentPosition();
        else {
            double currentPosition = rotator.getCurrentPosition();
            rotator.setPower(output.getOutput(currentPosition, targetPosition));
        }
        output.setKp(kp);
        output.setKi(ki);
        output.setKd(kd);
    }
    public void setLiftPosition (double liftPosition) {
        this.liftPosition = liftPosition;
    }


    public double getPosition() {
        return rotator.getCurrentPosition();
    }

    public double getRawPower () {
        return rotator.getPower();
    }

    public double getAngle() {
        double angle =  (rotator.getCurrentPosition() *
                rotator.getEncoder().getClicksPerRotation()) / 360;
        angle /= 170;
        if (angle < 0) angle = -angle;
        return angle;
    }

    public void setAngle (double angle, Direction direction, double timeout) {
        movementAllowed.set(true);
        MasqClock clock = new MasqClock();
        double angleRemaining = Math.abs(angle - Math.abs(getAngle()));
        while (angleRemaining > 5 && MasqRobot.opModeIsActive()
                && !clock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            angleRemaining = Math.abs(angle - Math.abs(getAngle()));
            double rawPower = angleRemaining / angle;
            rotator.setPower(rawPower * direction.value * 2.7);
            DashBoard.getDash().create(angleRemaining);
            DashBoard.getDash().update();
        }
        rotator.setPower(0);
        movementAllowed.set(false);
    }
    public void setAngle(double angle, Direction direction) {
        setAngle(angle, direction, 3);
    }

    @Override
    public String getName() {
        return null;
    }

    public void setMovementAllowed(boolean movementAllowed) {
        this.movementAllowed.set(movementAllowed);
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }

    @Override
    public void run() {
        close.set(true);
        while (close.get()) {
            kp = (1e-6 * -liftPosition) + 0.001;
            ki = 0.0;
            kd = 0.0;
            if (movementAllowed.get()) targetPosition = rotator.getCurrentPosition();
            else {
                double currentPosition = rotator.getCurrentPosition();
                rotator.setPower(output.getOutput(currentPosition, targetPosition));
            }
            output.setKp(kp);
            output.setKi(ki);
            output.setKd(kd);
        }
    }
    public void close() {close.set(false);}
    public void startHoldThread () {
        new Thread(this).start();
    }
}
