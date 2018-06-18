package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Robot extends MasqRobot {
    private HardwareMap hardwareMap;
    public MasqMotor leftMotor, rightMotor;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftMotor = new MasqMotor("leftMotor", this.hardwareMap);
        rightMotor = new MasqMotor("rightMotor", DcMotor.Direction.REVERSE, this.hardwareMap);
    }
}
