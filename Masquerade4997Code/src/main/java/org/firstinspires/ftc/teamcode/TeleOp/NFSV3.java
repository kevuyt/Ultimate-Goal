package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/10/17.
 */
@TeleOp(name = "NFSV3", group = "Autonomus")
public class NFSV3 extends MasqLinearOpMode implements Constants {
    double currentAdjusterPosition = 0;
    double increment = 0.05;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        boolean glyphBottomOpenState = true, jewelArmIn = true, clawClosed = true, glyphTopOpenState = true;
        robot.lift.setPositionLimits(LIFT_MIN, LIFT_MAX);
        robot.initializeServos();
        while (!opModeIsActive()){
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        robot.initializeTeleop();
        while (opModeIsActive()){
            robot.driveTrain.setClosedLoop(true);
            robot.MECH(controller1);
            if (controller1.aOnPress() && glyphBottomOpenState) {
                glyphBottomOpenState = false;
                robot.glyphSystemBottom.setPosition(GLYPH_CLOSED);
                controller1.update();
            }
            if (controller1.aOnPress() && !glyphBottomOpenState) {
                glyphBottomOpenState = true;
                robot.glyphSystemBottom.setPosition(GLYPH_OPENED);
                controller1.update();
            }
            if (controller1.bOnPress() && glyphTopOpenState) {
                glyphTopOpenState = false;
                robot.glyphSystemTop.setPosition(GLYPH_CLOSED);
                controller1.update();
            }
            if (controller1.bOnPress() && !glyphTopOpenState) {
                glyphTopOpenState = true;
                robot.glyphSystemTop.setPosition(GLYPH_OPENED);
                controller1.update();
            }
            if (controller1.xOnPress()) {
                robot.glyphSystemTop.setPosition(0.6);
                robot.glyphSystemBottom.setPosition(0.4);
                controller1.update();
            }
            if (controller2.start()) increment = 0.005;
            else increment = 0.05;
            if (controller2.xOnPress() && jewelArmIn) {
                jewelArmIn = false;
                robot.jewelArmRed.setPosition(JEWEL_OUT);
                controller2.update();
            } else if (controller2.xOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArmRed.setPosition(JEWEL_IN);
                controller2.update();
            }
            if (controller2.bOnPress() && jewelArmIn) {
                jewelArmIn = false;
                robot.jewelArmBlue.setPosition(JEWEL_OUT);
                controller2.update();
            } else if (controller2.bOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArmBlue.setPosition(JEWEL_IN);
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
//            dash.create(robot.matiboxUltraSensor);
//            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
//            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
//            dash.create("LIFT POSITION", robot.lift.getCurrentPosition());
//            dash.create("RELIC LIFT POSITION", robot.relicLift.getCurrentPosition());
//            dash.create("CURRENT POSITION: ",robot.relicAdjuster.getPosition());
//            dash.update();
        }
    }
}