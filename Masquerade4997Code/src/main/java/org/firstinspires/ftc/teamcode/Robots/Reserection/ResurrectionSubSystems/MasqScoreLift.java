package org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems;

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
public class MasqScoreLift extends MasqMotor implements MasqSubSystem {
    private MasqPIDController hold = new MasqPIDController(0.008, 0, 0);
    private double holdPosition = 0;
    public MasqScoreLift(String name, HardwareMap hardwareMap) {
        super(name, MasqMotorModel.ORBITAL20, hardwareMap);
        resetEncoder();
    }
    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            setPower(1);
            holdPosition = getCurrentPosition();
        }
        else if (controller.rightTriggerPressed()) {
            setPower(-1);
            holdPosition = getCurrentPosition();
        }
        else setPower(hold.getOutput(getCurrentPosition(), holdPosition));
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
