package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/8/17.
 */
@TeleOp(name = "NFSV2", group = "Template")
public class RelicADV extends MasqLinearOpMode implements Constants{
    @Override
    public void run() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.lift.setPositionLimits(-500, LIFT_MAX_ROTATIONS * TICKS_PER_ROTATION);
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            if (controller1.rightBumper()) {
                robot.leftGlyph.setPower(1);
                robot.rightGlyph.setPower(-1);
            } else if (controller1.leftBumper()) {
                robot.leftGlyph.setPower(-1);
                robot.rightGlyph.setPower(1);
            } else {
                robot.leftGlyph.setPower(0);
                robot.rightGlyph.setPower(0);
            }
            if (controller1.rightTriggerPressed()) robot.lift.setPower(controller1.rightTrigger());
            else if (controller1.leftTriggerPressed()) robot.lift.setPower(-1);
            else robot.lift.setPower(0);

            dash.create(controller1.a());
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.create("LIFT POSITION", robot.lift.getCurrentPosition());
            dash.update();
        }
    }
}
