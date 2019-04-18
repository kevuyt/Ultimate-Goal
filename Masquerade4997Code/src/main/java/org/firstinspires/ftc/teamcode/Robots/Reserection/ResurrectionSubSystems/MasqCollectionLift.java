package org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 4/14/19.
 * Project: MasqLib
 */
public class MasqCollectionLift implements MasqSubSystem {
    public MasqMotor lift;
    private MasqLimitSwitch liftSwitch;
    private MasqPIDController hold = new MasqPIDController(0.01, 0, 0);
    private double holdPosition = 0;
    public MasqCollectionLift(String name, HardwareMap hardwareMap) {
        lift = new MasqMotor(name, MasqMotorModel.ORBITAL20, hardwareMap);
        lift.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            lift.setPower(-1);
            holdPosition = lift.getCurrentPosition();
        }
        else if (controller.rightTriggerPressed() && !liftSwitch.isPressed()) {
            lift.setPower(1);
            holdPosition = lift.getCurrentPosition();
        }
        else /*lift.setPower(hold.getOutput(lift.getCurrentPosition(), holdPosition));*/ lift.setPower(0);
    }

    public void setLiftSwitch(MasqLimitSwitch liftSwitch) {
        this.liftSwitch = liftSwitch;
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
