package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/24/17.
 */
@TeleOp(name = "NFSV4", group = "Autonomus")
public class NFSV4 extends MasqLinearOpMode implements Constants {
    double currentAdjusterPosition = 0;
    double increment = 0.05;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        boolean glyphBottomOpenState = true, jewelArmIn = true, clawClosed = true, glyphTopOpenState = true;
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.driveTrain.setClosedLoop(true);
        while (opModeIsActive()) {
            robot.MECH(controller1);
            if (controller1.x()) {
                robot.glyphSystemBottom.setPosition(GLYPH_CLOSED);
            }
            if (controller1.a()) {
                robot.glyphSystemBottom.setPosition(GLYPH_OPENED);
            }
            if (controller1.y()) {
                robot.glyphSystemTop.setPosition(GLYPH_CLOSED);
            }
            if (controller1.b()) {
                robot.glyphSystemTop.setPosition(GLYPH_OPENED);
            }
            if (controller1.leftTriggerPressed()) {
                robot.glyphSystemTop.setPosition(0.6);
                robot.glyphSystemBottom.setPosition(0.4);
                controller1.update();
            }
            if (controller2.start()) increment = 0.005;
            else increment = 0.05;
            if (controller2.xOnPress() && jewelArmIn) {
                jewelArmIn = false;
                robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
                controller2.update();
            } else if (controller2.xOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArmRed.setPosition(JEWEL_RED_IN);
                controller2.update();
            }
            if (controller2.bOnPress() && jewelArmIn) {
                jewelArmIn = false;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
                controller2.update();
            } else if (controller2.bOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                controller2.update();
            }
            if (controller2.aOnPress() && clawClosed) {
                clawClosed = false;
                robot.relicGripper.setPosition(CLAW_OPENED);
                controller2.update();
            } else if (controller2.aOnPress() && !clawClosed) {
                clawClosed = true;
                robot.relicGripper.setPosition(CLAW_CLOSED);
                controller2.update();
            }
            if (controller2.leftBumper()) currentAdjusterPosition += increment;
            if (controller2.leftTriggerPressed()) currentAdjusterPosition -= increment;
            robot.relicAdjuster.setPosition(currentAdjusterPosition);
            if (controller1.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller1.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.rightBumper()) robot.relicLift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) {robot.relicLift.setPower(LIFT_DOWN);}
            else robot.relicLift.setPower(0);
            controller1.update();
            controller2.update();
        }
    }
}