package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/8/17.
 */
@TeleOp(name = "NFS", group = "Template")
public class RelicTeleOp extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            robot.leftGlyph.setPosition((gamepad1.left_stick_x/2) + .5);
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.update();
        }
    }
}
