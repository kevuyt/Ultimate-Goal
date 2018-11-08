package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqSubSystem;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem {
    public MasqMotorSystem rotator;
    private double targetPosition;
    private double basePower = 0.3;
    private double baseDownPower = 0.1;
    private double rotatorPower = basePower;
    private double downPower = -0.1;
    private double liftPosition = 0;
    private double kp = 0.01, ki, kd;
    public MasqPIDController output = new MasqPIDController(0.01, 0.0, 0.00);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotatorOne", DcMotor.Direction.FORWARD,"rotatorTwo",
                DcMotor.Direction.REVERSE, "rotator", hardwareMap, MasqMotorModel.NEVEREST60);
        rotator.setClosedLoop(true);
        rotator.resetEncoders();
    }
    @Override
    public void DriverControl(MasqController controller) {
        kp = (1e-6 * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * -liftPosition) + basePower;
        downPower = (0.001 * Math.abs(rotator.getCurrentPosition())) + baseDownPower;
        if (controller.leftTriggerPressed()) rotator.setPower(rotatorPower);
        else if (controller.rightTriggerPressed()) rotator.setPower(-downPower);

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
        return rotator.getAngle();
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
