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
        int num = 0;
        boolean glyphOpenState = true, jewelArmIn = true;
        robot.lift.setPositionLimits(790, LIFT_MAX_ROTATIONS * TICKS_PER_ROTATION);
        while (!opModeIsActive()){
            dash.create(num);
            dash.create(controller1.a());
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
                controller1.update();
            } else if (controller1.bOnPress() && !jewelArmIn) {
                jewelArmIn = true;
                robot.jewelArm.setPosition(JEWEL_IN);
                controller1.update();
            } else robot.jewelArm.setPosition(JEWEL_OUT);
            if (controller1.aOnPress()) num++;
            if (controller1.rightTriggerPressed()) robot.lift.setPower(controller1.rightTrigger());
            else if (controller1.leftTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            controller1.update();
            dash.create(num);
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.create("LIFT POSITION", robot.lift.getCurrentPosition());
            dash.update();
        }
    }
}
