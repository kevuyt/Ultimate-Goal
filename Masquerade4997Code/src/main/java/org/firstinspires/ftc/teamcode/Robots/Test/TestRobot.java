package org.firstinspires.ftc.teamcode.Robots.Test;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;

/**
 * Created by Keval Kataria on 9/25/2019
 */
public class TestRobot extends MasqRobot {
    MasqMotor masqMotor;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        masqMotor = new MasqMotor("testMotor", MasqMotorModel.ORBITAL20,hardwareMap);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}
