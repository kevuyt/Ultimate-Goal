package org.firstinspires.ftc.teamcode.Robots.WebcamBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/16/2019
 */
@TeleOp(name = "WebcamTeleop", group = "WebcamBot")
public class WebcamTeleop extends MasqLinearOpMode {
    private WebcamBot robot = new WebcamBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        robot.detector.start();
        while (opModeIsActive()) {
            sleep(0.1);
        }
    }
}
