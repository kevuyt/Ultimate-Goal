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
        while (!opModeIsActive() || opModeIsActive()) {
            robot.init(hardwareMap);
            robot.detector.start();
            dash.create(robot.detector.getStoneSkystonePosition());
            dash.update();
        }
    }
}
