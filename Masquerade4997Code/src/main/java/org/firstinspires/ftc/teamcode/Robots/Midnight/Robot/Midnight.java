package org.firstinspires.ftc.teamcode.Robots.Midnight.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
public class Midnight extends MasqRobot {
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        MasqMotorSystem left = new MasqMotorSystem(new MasqMotor("left", MasqMotorModel.ORBITAL20, DcMotor.Direction.FORWARD, hardwareMap));
        MasqMotorSystem right = new MasqMotorSystem(new MasqMotor("right", MasqMotorModel.ORBITAL20, DcMotor.Direction.REVERSE, hardwareMap));
        driveTrain = new MasqMechanumDriveTrain(left, right);
    }

    @Override
    public void init(HardwareMap hardwareMap) throws InterruptedException {

    }
}
