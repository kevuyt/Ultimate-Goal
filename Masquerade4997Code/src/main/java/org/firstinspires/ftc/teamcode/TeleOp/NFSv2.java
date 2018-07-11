package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

@TeleOp(name = "NFSV2", group = "NFS")
public class NFSv2 extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Init");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}