package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.TestBot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

@TeleOp(name = "NFS", group = "NFS")
public class NFS extends MasqLinearOpMode implements Constants {
    private TestBot robot = new TestBot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Init");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.NFS(controller1);
        }
    }
}
