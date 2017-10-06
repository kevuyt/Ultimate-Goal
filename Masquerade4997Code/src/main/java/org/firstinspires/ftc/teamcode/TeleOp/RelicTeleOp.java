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
        //Make hardware map and set minimum limit switch
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()){
            dash.create(robot.imu.getHeading());
            dash.create(controller1.a());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){

            //Set robot to NFS
            robot.NFS(controller1);

            //Position setting of the glyph servos
            robot.leftGlyph.setPosition((controller1.leftTrigger()/2) + .5);
            robot.rightGlyph.setPosition((-controller1.leftTrigger()/2) - .5);

            //Set position of lift if within usable range and not pressing min limit switch
            if (controller1.rightTrigger() > 0 &&
                    robot.lift.getCurrentPosition() <= 1000)
                        robot.lift.setPower(1);

            //Set power zero if position is greater than usable range using getCurrentPosition
            else if (controller1.rightTrigger() == 0 || robot.lift.getCurrentPosition() <= 0) robot.lift.setPower(0);

            //If trigger isn't pressed move down until limit switch is pressed
            else robot.lift.setPower(-1);

            //dash for teleop
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.update();
        }
    }
}
