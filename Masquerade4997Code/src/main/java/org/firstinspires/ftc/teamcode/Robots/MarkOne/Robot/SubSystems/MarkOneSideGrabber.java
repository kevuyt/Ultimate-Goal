package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 2020-01-01.
 * Project: MasqLib
 */
public class MarkOneSideGrabber implements MasqSubSystem{
    public MasqServoSystem sideGrabber;

    public MarkOneSideGrabber(HardwareMap hardwareMap, String name1, String name2) {
        sideGrabber = new MasqServoSystem(name1, name2, hardwareMap);
        sideGrabber.scaleRange(0,1);
        sideGrabber.servo1.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void DriverControl(MasqController controller) throws InterruptedException {
        if (controller.dPadLeft()) down();
        else up();
    }

    public void up() throws InterruptedException {
        sideGrabber.servo1.setPosition(0);
        Thread.sleep(1000);
        sideGrabber.servo1.setPosition(0);
    }
    public void down() throws InterruptedException {
        sideGrabber.servo1.setPosition(1);
        Thread.sleep(1000);
        sideGrabber.servo1.setPosition(1);
    }
    public void close() {
        sideGrabber.servo2.setPosition(0);
    }
    public void open() {
        sideGrabber.servo2.setPosition(1);
    }

    @Override
    public String getName() {
        return "SideGrabber";
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
