package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/8/17.
 */
@TeleOp(name = "NFSV2", group = "Template")
public class RelicRecovery extends MasqLinearOpMode implements Constants {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        boolean glyphOpenState = true, jewelArmIn = true, clawClosed = true;
        robot.lift.setPositionLimits(LIFT_MIN, LIFT_MAX);
        robot.initalizeServos();
        while (!opModeIsActive()){
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            if (controller1.aOnPress() && glyphOpenState) {
                glyphOpenState = false;
                robot.glyphSystem.setPosition(GLYPH_CLOSED);
                controller1.update();
            } if (controller1.aOnPress() && !glyphOpenState) {
                glyphOpenState = true;
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                controller1.update();
            }
            if (controller1.bOnPress() && jewelArmIn) {
                jewelArmIn = false;
                robot.jewelArm.setPosition(JEWEL_OUT);
                controller2.update();
            } else if (controller1.bOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArm.setPosition(JEWEL_IN);
                controller1.update();
            }
            if (controller1.xOnPress() && clawClosed) {
                clawClosed = false;
                robot.relicGripper.setPosition(CLAW_OPENED);
                controller2.update();
            } else if (controller1.xOnPress() && !clawClosed) {
                clawClosed = true;
                robot.relicGripper.setPosition(CLAW_CLOSED);
                controller1.update();
            }
            if (controller1.rightTriggerPressed()) robot.lift.setPower(controller1.rightTrigger());
            else if (controller1.leftTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller1.rightBumper()) robot.relicLift.setPower(LIFT_UP);
            else if (controller1.leftBumper()) robot.relicLift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            controller1.update();
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.create("LIFT POSITION", robot.lift.getCurrentPosition());
            dash.create("RELIC LIFT POSITION", robot.relicLift.getCurrentPosition());
            dash.update();
        }
    }
}
