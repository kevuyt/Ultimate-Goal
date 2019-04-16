package org.firstinspires.ftc.teamcode.Robots.Reserection.FalconSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 4/15/19.
 * Project: MasqLib
 */
public class MasqCollectorDumper implements MasqSubSystem {
    private MasqPIDController hold = new MasqPIDController(0.01, 0, 0);
    private MasqMotor dumper;
    public MasqCollectorDumper (String name, HardwareMap hardwareMap) {
        dumper = new MasqMotor(name, MasqMotorModel.ORBITAL20, hardwareMap);
    }
    @Override
    public void DriverControl(MasqController controller) {
        if (controller.y()) dumper.setPower(1);
        else if (controller.x()) dumper.setPower(-1);
        else dumper.setPower(0);
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
