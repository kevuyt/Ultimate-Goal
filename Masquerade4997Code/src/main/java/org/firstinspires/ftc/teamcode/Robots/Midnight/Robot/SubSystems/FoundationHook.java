package org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.Constants;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.LEFT_DOWN;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.LEFT_UP;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.RIGHT_DOWN;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.RIGHT_UP;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
public class FoundationHook implements MasqSubSystem, Constants {
    private MasqServo leftHook, rightHook;
    public FoundationHook(HardwareMap hardwareMap) {
        leftHook = new MasqServo("leftHook", hardwareMap);
        rightHook = new MasqServo("rightHook", hardwareMap);
    }
    @Override
    public void DriverControl(MasqController controller) {
        if (controller.a()) hooksDown();
        else hooksUp();
    }

    public void hooksDown() {
        leftHook.setPosition(LEFT_DOWN);
        rightHook.setPosition(RIGHT_DOWN);
    }

    public void hooksUp() {
        leftHook.setPosition(LEFT_UP);
        rightHook.setPosition(RIGHT_UP);
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
