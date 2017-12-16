package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/11/17.
 */
@TeleOp(name = "NFSV4", group = "Autonomus")
public class NFSV4 extends MasqLinearOpMode implements Constants {
    @Override
     public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        boolean glyphBottomOpenState = true, jewelArmInRed = true, jewelArmInBlue = true, clawClosed = true, glyphTopOpenState = true;
        boolean  bottomLimit, topLimit, liftLimit, liftLimitPrressed = true;
        int glyphCount = 0;
        robot.initializeServos();
        while (!opModeIsActive()){
            dash.create(INIT_MESSAGE);
            dash.create(robot.bottomLimit.getState());
            dash.update();
        }
        waitForStart();
        robot.initializeTeleop();
        robot.lift.runUsingEncoder();
        robot.lift.setLimit(robot.liftLimit);
        while (opModeIsActive()){
            if (robot.glyphSystemTop.getPosition() == GLYPH_TOP_CLOSED) topLimit = true;
            else topLimit = false;
            liftLimit = robot.liftLimit.getState();
            bottomLimit = robot.bottomLimit.getState();
            robot.driveTrain.setClosedLoop(true);
            robot.MECH(controller1);
            if (controller1.aOnPress() && glyphBottomOpenState) {
                glyphBottomOpenState = false;
                robot.bottomIntake.setPower(0);
                robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_OPENED);
                controller1.update();
            }
            if (controller1.aOnPress() && !glyphBottomOpenState) {
                glyphBottomOpenState = true;
                robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_CLOSED);
                robot.bottomIntake.setPower(INTAKE);
                controller1.update();
            }
            if (controller1.bOnPress() && glyphTopOpenState) {
                glyphTopOpenState = false;
                robot.glyphSystemTop.setPosition(GLYPH_TOP_CLOSED);
                controller1.update();
            }
            if (controller1.bOnPress() && !glyphTopOpenState) {
                glyphTopOpenState = true;
                robot.glyphSystemTop.setPosition(GLYPH_TOP_OPENED);
                controller1.update();
            }
            if (controller1.xOnPress()) {
                robot.glyphSystemTop.setPosition(0.6);
                robot.glyphSystemBottom.setPosition(0.4);
                glyphBottomOpenState = false;
                glyphTopOpenState = true;
                robot.bottomIntake.setPower(0);
                controller1.update();
            }
            if (controller2.xOnPress() && jewelArmInRed) {
                jewelArmInRed = false;
                robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
                controller2.update();
            }
            else if (controller2.xOnPress() && !jewelArmInRed) {
                jewelArmInRed = true;
                robot.jewelArmRed.setPosition(JEWEL_RED_IN);
                controller2.update();
            }
            if (controller2.bOnPress() && jewelArmInBlue) {
                jewelArmInBlue = false;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
                controller2.update();
            }
            else if (controller2.bOnPress() && !jewelArmInBlue) {
                jewelArmInBlue = true;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                controller2.update();
            }
            if (controller2.aOnPress() && clawClosed) {
                clawClosed = false;
                robot.relicGripper.setPosition(CLAW_OPENED);
                controller2.update();
            }
            else if (controller2.aOnPress() && !clawClosed) {
                clawClosed = true;
                robot.relicGripper.setPosition(CLAW_CLOSED);
                controller2.update();
            }
            if (controller2.leftBumper()) robot.relicAdjuster.setPower(0.5);
            else if (controller2.leftTriggerPressed()) robot.relicAdjuster.setPower(-0.5);
            else robot.relicAdjuster.setPower(0);
            if (controller1.rightBumper()) {
                liftLimitPrressed = false;
                robot.lift.setPower(LIFT_UP);
                robot.lift.setLazy();
            }
            else if (controller1.rightTriggerPressed()) {
                robot.lift.setPower(LIFT_DOWN);
                robot.lift.setLazy();
            }
            else {
                robot.lift.setPower(0);
                robot.lift.setStrong();
            }
            if (bottomLimit && !topLimit) {
                robot.bottomIntake.setPower(0);
                robot.glyphSystemTop.setPosition(GLYPH_TOP_CLOSED);
                robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_OPENED);
            }
            if (bottomLimit && topLimit) {
                robot.bottomIntake.setPower(0);
                robot.glyphSystemTop.setPosition(GLYPH_TOP_CLOSED);
            }
            if (liftLimit) {
                liftLimitPrressed = true;
                robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_CLOSED);
                robot.glyphSystemTop.setPosition(GLYPH_TOP_OPENED);
                robot.bottomIntake.setPower(INTAKE);
            }
            if (liftLimitPrressed) robot.lift.setPower(0);
            //robot.lift.doHold();
            if (controller2.rightBumper()) robot.relicLift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) {robot.relicLift.setPower(LIFT_DOWN);}
            else robot.relicLift.setPower(0);
            dash.create("Glyph Count: " + glyphCount);
            dash.update();
            controller1.update();
            controller2.update();
            robot.sleep(20);
        }
    }
}