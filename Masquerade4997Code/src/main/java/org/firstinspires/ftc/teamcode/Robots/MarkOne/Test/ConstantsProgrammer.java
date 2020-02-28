package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-12.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "MarkOne")
public class ConstantsProgrammer extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
    private double capPos, speed, rightGrabber, leftRotator, rightRotator, rotator;
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello, this is a constants programmer.");
            dash.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            if (controller1.a()) capPos += 0.001;
            else if (controller1.b()) capPos -= 0.001;

            if (controller1.dPadUp()) speed += 0.001;
            else if (controller1.dPadDown()) speed -= 0.001;

            if (controller1.dPadRight()) rightGrabber += 0.001;
            else if (controller1.dPadLeft()) rightGrabber -= 0.001;

            if (controller1.leftBumper()) leftRotator += 0.001;
            else if (controller1.leftTriggerPressed()) leftRotator -= 0.001;

            if (controller1.rightBumper()) rightRotator += 0.001;
            else if (controller1.rightTriggerPressed()) rightRotator -= 0.001;

            if (controller2.rightBumper()) rotator += 0.001;
            else if (controller2.rightTriggerPressed()) rotator -= 0.001;

            speed = Range.clip(speed, 0 , 1);
            rightGrabber = Range.clip(rightGrabber, 0 , 1);
            leftRotator = Range.clip(leftRotator, 0 , 1);
            rightRotator = Range.clip(rightRotator, 0 , 1);
            capPos = Range.clip(capPos,0,1);
            rotator = Range.clip(rotator, 0, 1);

            robot.sideGrabber.rightGrabber.setPosition(rightGrabber);
            robot.sideGrabber.leftGrabber.setPosition(speed);
            robot.sideGrabber.rightRotater.setPosition(rightRotator);
            robot.sideGrabber.leftRotater.setPosition(leftRotator);
            robot.blockRotater.setPosition(rotator);
            robot.capper.setPosition(capPos);

            //robot.driveTrain.setPower(speed);
            dash.create("Rotator (+D_UP, -D_DOWN): ", rotator);
            dash.create("Left Grabber (+D_UP, -D_DOWN): ", speed);
            dash.create("Right Grabber (+D_RIGHT, -D_LEFT): ", rightGrabber);
            dash.create("Left Rotator (+LB, -LT): ", leftRotator);
            dash.create("Right Rotator (+RB, -RT): ", rightRotator);
            dash.create("Hook Position: ", capPos);
            dash.update();
        }
    }
}
