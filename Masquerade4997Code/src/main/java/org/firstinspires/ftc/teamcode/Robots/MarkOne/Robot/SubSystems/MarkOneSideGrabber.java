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
        //leftRotater.setDirection(Servo.Direction.REVERSE);
        leftGrabber.setDirection((Servo.Direction.REVERSE));
    }

    @Override
    public void DriverControl(MasqController controller) {}

    public void scaleServos() {
        leftRotater.scaleRange(0.1, 0.5);
        leftGrabber.scaleRange(0.24, 0.75);
        rightRotater.scaleRange(0.23, 0.65);
        rightGrabber.scaleRange(0.07, 0.6);
    }

    public void leftUp(double sleepTime) {
        leftRotater.setPosition(1);
        sleep(sleepTime);
    }
    public void leftDown(double sleepTime) {
        leftRotater.setPosition(0);
        sleep(sleepTime);
    }
    public void leftLowMid(double sleepTime) {
        leftRotater.setPosition(0.45);
        sleep(sleepTime);
    }
    public void leftClose(double sleepTime) {
        leftGrabber.setPosition(1);
        sleep(sleepTime);
    }
    public void leftSlightClose(double sleepTime) {
        leftGrabber.setPosition(0.5);
        sleep(sleepTime);
    }
    public void leftOpen(double sleepTime) {
        leftGrabber.setPosition(0);
        sleep(sleepTime);
    }
    public void leftMid(double sleepTime) {
        leftRotater.setPosition(0.15);
        sleep(sleepTime);
    }
    public void rightUp(double sleepTime) {
        rightRotater.setPosition(0.1);
        sleep(sleepTime);
    }
    public void rightMid(double sleepTime) {
        rightRotater.setPosition(0.2);
        sleep(sleepTime);
    }
    public void rightDown(double sleepTime) {
        rightRotater.setPosition(1);
        sleep(sleepTime);
    }
    public void rightLowMid(double sleepTime) {
        rightRotater.setPosition(0.45);
        sleep(sleepTime);
    }
    public void rightClose(double sleepTime) {
        rightGrabber.setPosition(1);
        sleep(sleepTime);
    }
    public void rightSlightClose(double sleepTime) {
        rightGrabber.setPosition(0.5);
        sleep(sleepTime);
    }
    public void rightOpen(double sleepTime) {
        rightGrabber.setPosition(0);
        sleep(sleepTime);
    }
    public void reset() {
        leftUp(0);
        rightUp(1);
        leftOpen(0);
        rightOpen(0);
    }

    public void teleReset() {
        leftUp(0);
        rightUp(0);
        leftClose(0);
        rightClose(0);
    }

    public void getSet() {
        leftRotater.setPosition(leftRotater.getPosition());
        leftGrabber.setPosition(leftGrabber.getPosition());
        rightRotater.setPosition(rightRotater.getPosition());
        rightGrabber.setPosition(rightGrabber.getPosition());
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