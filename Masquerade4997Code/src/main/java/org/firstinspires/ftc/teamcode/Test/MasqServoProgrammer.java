package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;
import SubSystems4997.SubSystems.Gripper;

/**
 * Created by Archish on 12/8/17.
 */
@TeleOp(name = "MasqServoProgrammer", group = "T")
public class MasqServoProgrammer extends MasqLinearOpMode implements Constants {
    double rightFlipper = Flipper.Position.IN.pos[0], leftFlipper = Flipper.Position.IN.pos[1],
            bottomGripper = Gripper.Grip.CLAMP.grip[0], topGripper = Gripper.Grip.CLAMP.grip[1],
            relicGrabber = CLAW_CLOSED, relicWrist = 0,
           jewelRed, redRotator;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        while (opModeIsActive()) {
            if  (controller2.bOnPress()) {
                rightFlipper += 0.01;
                controller2.update();
            }
            if (controller2.xOnPress()) {
                rightFlipper -= 0.01;
                controller2.update();
            }
            if (controller2.yOnPress()) {
                leftFlipper -= 0.01;
                controller2.update();
            }
            if (controller2.aOnPress()) {
                leftFlipper += 0.01;
                controller2.update();
            }
            if (controller2.leftTriggerOnPress()) {
                bottomGripper += 0.01;
                controller2.update();
            }
            if (controller2.leftBumperOnPress()) {
                bottomGripper -= 0.01;
                controller2.update();
            }
            if (controller2.rightTriggerOnPress()) {
                topGripper += 0.01;
                controller2.update();
            }
            if (controller2.rightBumperOnPress()) {
                topGripper -= 0.01;
                controller2.update();
            }
            if (controller1.aOnPress()) {
                relicGrabber += 0.01;
                controller1.update();
            }
            if (controller1.xOnPress()) {
                relicGrabber -= 0.01;
                controller1.update();
            }
            if (controller1.bOnPress()) {
                relicWrist += 0.01;
                controller1.update();
            }
            if (controller1.yOnPress()) {
                relicWrist -= 0.01;
                controller1.update();
            }
            if (controller1.rightTriggerPressed()) {
                jewelRed += 0.01;
                controller1.update();
            }
            if (controller1.rightBumper()) {
                jewelRed -= 0.01;
                controller1.update();
            }
            if (controller1.leftTriggerPressed()) {
                redRotator += 0.01;
                controller1.update();
            }
            if (controller1.leftBumper()) {
                redRotator -= 0.01;
                controller1.update();
            }
            robot.jewelArmRed.setPosition(jewelRed);
            robot.redRotator.setPosition(redRotator);
            robot.gripper.gripTop.setPosition(topGripper);
            robot.gripper.gripBottom.setPosition(bottomGripper);
            robot.flipper.flipperLeft.setPosition(leftFlipper);
            robot.flipper.flipperRight.setPosition(rightFlipper);
            robot.relicAdjuster.setPosition(relicWrist);
            robot.relicGripper.setPosition(relicGrabber);
            dash.create("FLIP RIGHT (C2 X, B): ", rightFlipper);
            dash.create("FLIP LEFT (C2 Y, A): ", leftFlipper);
            dash.create("GRIP TOP: (C2 RT, RB)", topGripper);
            dash.create("GRIP BOTTOM (C2 LT, LB) : ", bottomGripper);
            dash.create("RELIC WRIST (C1 B, Y): ", relicWrist);
            dash.create("RELIC GRABBER: (C1 A, X)", relicGrabber);
            dash.create("RED ROTATOR: (C1, RT, RB) ", redRotator);
            dash.create("RED JEWEL (C1, RT, RB): ", jewelRed);
            dash.update();
            controller1.update();
            controller2.update();
            robot.sleep(50);
        }
    }
}