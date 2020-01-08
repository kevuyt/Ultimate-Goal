package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.Constants;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

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
        sideGrabber.servo1.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void DriverControl(MasqController controller) {
    }

    public void scaleServos() {
        leftRotater.scaleRange(0.5, 1);
        leftGrabber.scaleRange(0.5, 0.85);
        rightRotater.scaleRange(0.1,0.7);
        rightGrabber.scaleRange(0.2,0.6);}
    public void leftUp() {
        sideGrabber.servo1.setPosition(0);
    }
    public void leftDown() {
        sideGrabber.servo1.setPosition(1);
    }
    public void leftClose() {
        sideGrabber.servo2.setPosition(1);
    }
    public void leftOpen() {
        sideGrabber.servo2.setPosition(0);
    }

    public void rightUp() {
        sideGrabber.servo3.setPosition(0);
    }
    public void rightMid() {
        sideGrabber.servo3.setPosition(0.35);
    }
    public void rightDown() {
        sideGrabber.servo3.setPosition(1);
    }
    public void rightClose() {
        sideGrabber.servo4.setPosition(1);
    }
    public void rightSlightClose() {sideGrabber.servo4.setPosition(0.625);}
    public void rightOpen() {
        sideGrabber.servo4.setPosition(0);
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