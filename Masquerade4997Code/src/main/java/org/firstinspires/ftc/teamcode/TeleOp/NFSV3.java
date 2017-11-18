package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/10/17.
 */
@TeleOp(name = "NFSV3", group = "Autonomus")
public class NFSV3 extends MasqLinearOpMode implements Constants {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapMotors(hardwareMap);
        robot.initializeTeleop();
        while (!opModeIsActive()){
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.MECH(controller1);
            dash.create("LEFT",robot.driveTrain.leftDrive.getRate());
            dash.create("RIGHT", robot.driveTrain.rightDrive.getRate());
            dash.update();
        }
    }
}