package org.firstinspires.ftc.teamcode.Robots.Reserection.ResurrectionSubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 4/17/19.
 * Project: MasqLib
 */
public class MasqCollectorDumper extends MasqMotor implements MasqSubSystem {
    private MasqPIDController collectionDumpHold = new MasqPIDController(0.02, 0, 0);
    private double transferPosition = 0, passTransferPosition = 0, downPosition = 0;
    private double targetPosition = 0;

    public MasqCollectorDumper(String name, HardwareMap hardwareMap) {
        super(name, MasqMotorModel.NEVEREST60, hardwareMap);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.dPadUp()) targetPosition = transferPosition;
        else if (controller.dPadDown()) targetPosition = downPosition;
        else if (controller.dPadRight()) targetPosition = passTransferPosition;
        else setPower(collectionDumpHold.getOutput(getCurrentPosition(), targetPosition));
    }


    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
