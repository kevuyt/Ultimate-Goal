package org.firstinspires.ftc.teamcode.Robots.Reserection.FalconSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 4/14/19.
 * Project: MasqLib
 */
public class MasqCollectionLift implements MasqSubSystem {
    private MasqMotor lift;
    private MasqPIDController hold = new MasqPIDController(0.01, 0, 0);
    private double holdPosition = 0;
    public MasqCollectionLift(String name, HardwareMap hardwareMap) {
        lift = new MasqMotor(name, MasqMotorModel.ORBITAL20, hardwareMap);
        lift.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.leftBumper()) {
            lift.setPower(1);
            holdPosition = lift.getCurrentPosition();
        }
        else if (controller.leftTriggerPressed()) {
            lift.setPower(-1);
            holdPosition = lift.getCurrentPosition();
        }
        else lift.setPower(hold.getOutput(lift.getCurrentPosition(), holdPosition));
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
