package org.firstinspires.ftc.teamcode.Robots.TestRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;

/**
 * Created by Keval Kataria on 9/25/2019
 */
public class TestRobot extends MasqRobot {
    public MasqMotorSystem intake;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, MasqMotorModel.ORBITAL20);
        //intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE, MasqMotorModel.ORBITAL20,hardwareMap);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);

    }

}