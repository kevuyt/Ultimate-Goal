package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.Constants;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

import static Library4997.MasqResources.MasqUtils.sleep;

/**
 * Created by Archishmaan Peyyety on 2020-01-01.
 * Project: MasqLib
 */
public class MarkOneSideGrabber implements MasqSubSystem, Constants {
    public MasqServoSystem sideGrabber;
    public MasqServo leftRotater, leftGrabber, rightRotater, rightGrabber;

    public MarkOneSideGrabber(HardwareMap hardwareMap) {
        sideGrabber = new MasqServoSystem("leftAutoRotater", "leftAutoGrabber", "rightAutoRotater", "rightAutoGrabber", hardwareMap);
        leftRotater = sideGrabber.servo1;
        leftGrabber = sideGrabber.servo2;
        rightRotater = sideGrabber.servo3;
        rightGrabber = sideGrabber.servo4;
        leftRotater.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void DriverControl(MasqController controller) {
    }

    public void scaleServos() {
        leftRotater.scaleRange(0.5, 1);
        leftGrabber.scaleRange(0.5, 0.85);
        rightRotater.scaleRange(0.1,0.7);
        rightGrabber.scaleRange(0.2,0.65);
    }

    public void leftUp() {
        leftRotater.setPosition(0);
    }
    public void leftDown() {
        leftRotater.setPosition(1);
    }
    public void leftClose() {
        leftGrabber.setPosition(1);
        sleep(1.);
    }
    public void leftOpen() {
        leftGrabber.setPosition(0);
    }

    public void rightUp() {
        rightRotater.setPosition(0);
    }
    public void rightMid() {
        rightRotater.setPosition(0.35);
        sleep(1.);
    }
    public void rightDown() {
        rightRotater.setPosition(1);
        sleep(1.);
    }
    public void rightClose() {
        rightGrabber.setPosition(1);
        sleep(1.);
    }
    public void rightSlightClose() {
        rightGrabber.setPosition(0.625);
        sleep(1.);
    }
    public void rightOpen() {
        rightGrabber.setPosition(0);
    }
    public void reset() {
        leftUp();
        rightUp();
        leftOpen();
        rightOpen();
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