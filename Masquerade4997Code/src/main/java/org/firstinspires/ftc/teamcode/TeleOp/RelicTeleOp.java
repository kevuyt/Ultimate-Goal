package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/8/17.
 */
@TeleOp(name = "NFS", group = "Template")
public class RelicTeleOp extends MasqLinearOpMode implements Constants{
    @Override
    public void run() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.lift.setPositionLimits(0, LIFT_MAX_ROTATIONS * TICKS_PER_ROTATION);
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            robot.leftGlyph.setPower((gamepad1.left_trigger));
            robot.rightGlyph.setPower(-((gamepad1.left_trigger)));

            robot.lift.setPower(controller1.rightTrigger());
            if (!controller1.rightTriggerPressed()) robot.lift.setPower(-1);

            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.create("LIFT POSITION", robot.lift.getCurrentPosition());
            dash.update();
        }
    }
}
