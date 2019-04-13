package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 3/30/19.
 * Project: MasqLib
 */
public class MasqElevator implements MasqSubSystem {
    public MasqMotor lift;
    private MasqPIDController pidController = new MasqPIDController(0.001, 0, 0.00001);
    private double targetPosition;
    public MasqElevator (HardwareMap hardwareMap) {
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, DcMotor.Direction.REVERSE, hardwareMap);
        lift.resetEncoder();
        lift.setClosedLoop(true);
        lift.setKp(0.001);
    }
    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            targetPosition = lift.getCurrentPosition();
            lift.setVelocity(-1);
        }
        else if (controller.rightTriggerPressed()) {
            targetPosition = lift.getCurrentPosition();
            lift.setVelocity(1);
        }
        else lift.setVelocity(pidController.getOutput(lift.getCurrentPosition(), targetPosition));
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    @Override
    public String getName() {
        return null;
    }
    public void setKp(double kp) {
        pidController.setKp(kp);
    }
    public void setKi(double ki) {
        pidController.setKi(ki);
    }
    public void setKd(double kd) {
        pidController.setKd(kd);
    }
    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
