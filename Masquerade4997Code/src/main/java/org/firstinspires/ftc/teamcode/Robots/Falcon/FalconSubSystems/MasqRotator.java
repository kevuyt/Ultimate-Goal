package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem {
    public MasqMotorSystem rotator;
    private MasqLimitSwitch limitSwitch;
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotator1", DcMotor.Direction.FORWARD, "rotator2", DcMotor.Direction.REVERSE, MasqMotorModel.NEVEREST60, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.resetEncoders();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) rotator.setPower(-1);
        else if (controller.rightTriggerPressed()) rotator.setPower(1);
        else {
            rotator.setPower(0);
            rotator.setBreakMode();
        }
    }

    public void setLimitSwitch(MasqLimitSwitch limitSwitch) {
        this.limitSwitch = limitSwitch;
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
