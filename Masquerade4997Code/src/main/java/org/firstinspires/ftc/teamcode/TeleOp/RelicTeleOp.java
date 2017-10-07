package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/8/17.
 */
@TeleOp(name = "NFS", group = "Template")
public class RelicTeleOp extends MasqLinearOpMode {
    @Override
    public void run() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);

            robot.leftGlyph.setPosition((controller1.leftTrigger()/2) + .5);
            robot.rightGlyph.setPosition((-controller1.leftTrigger()/2) - .5);

            if (controller1.rightTrigger() > 0 &&
                    robot.lift.getCurrentPosition() <= 1000)
                        robot.lift.setPower(1);
            else if (controller1.rightTrigger() == 0 || robot.lift.getCurrentPosition() <= 0) robot.lift.setPower(0);
            else robot.lift.setPower(-1);

            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.update();
        }
    }
}
