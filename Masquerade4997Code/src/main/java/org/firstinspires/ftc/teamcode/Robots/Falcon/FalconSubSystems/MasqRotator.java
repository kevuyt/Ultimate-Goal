package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class MasqRotator implements MasqSubSystem {
    public MasqMotorSystem rotator;
    double targetPosition = 0;
    private MasqPIDController pidController = new MasqPIDController(0.001, 0, 0);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotator1", DcMotor.Direction.FORWARD, "rotator2", DcMotor.Direction.REVERSE, MasqMotorModel.NEVERREST256, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.setKp(0.01);
        rotator.resetEncoders();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            rotator.setVelocity(-1);
            targetPosition = rotator.getCurrentPosition();
        }
        else if (controller.rightTriggerPressed()) {
            rotator.setVelocity(1);
            targetPosition = rotator.getCurrentPosition();
        }
        else rotator.setVelocity(pidController.getOutput(rotator.getCurrentPosition(), targetPosition));
    }

    public double getAngle() {
        double angle = -(rotator.motor2.getCurrentPosition() / 4400) * 360;
        if (angle < 0) return 0;
        else if (angle > 100) return 100;
        return angle;
    }

    public double getProportionalPower(double angle, double min) {
        double a = (min - 1) / 3025;
        double b = -(2 * (min - 1)) / 55;
        double y = (a * angle * angle) + (b * angle) + min;
        if (y < min) return min;
        return y;
    }

    @Override
    public String getName() {
        return null;
    }


    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
