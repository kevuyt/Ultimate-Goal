package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.atomic.AtomicBoolean;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem, Runnable {
    public MasqMotorSystem rotator;
    private double targetPosition;
    private AtomicBoolean close = new AtomicBoolean(false);
    private AtomicBoolean movementAllowed = new AtomicBoolean(true);
    private double liftPosition = 0;
    private double kp = 0.01, ki, kd;
    public MasqPIDController output = new MasqPIDController(0.03, 0.0, 0.00);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotator1", "rotator2", MasqMotorModel.NEVEREST60, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.resetEncoders();
    }

    @Override
    public void DriverControl(MasqController controller) {
        kp = (1e-6 * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        if (controller.leftTriggerPressed()) rotator.setPower(1);
        else if (controller.rightTriggerPressed()) rotator.setPower(-1);

        if (controller.rightTrigger() > 0 || controller.leftTrigger() > 0) targetPosition = rotator.getCurrentPosition();
        else {
            rotator.setPower(0);
            rotator.setBreakMode();
            /*double currentPosition = rotator.getCurrentPosition();
            rotator.setPower(output.getOutput(currentPosition, targetPosition));*/
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
