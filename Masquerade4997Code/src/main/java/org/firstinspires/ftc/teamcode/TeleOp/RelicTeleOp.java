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
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            if (controller1.a()) {
                robot.crServoOne.setPower(1);
                robot.crServoTwo.setPower(1);
            } else if (controller1.b()) {
                robot.crServoOne.setPower(-1);
                robot.crServoTwo.setPower(-1);
            } else {
                robot.crServoOne.setPower(0);
                robot.crServoTwo.setPower(0);
            }
            robot.crServoTwo.setPower(1);
            dash.update();
        }
    }
}
