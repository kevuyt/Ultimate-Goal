package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp
public class WebcamTeleop  extends MasqLinearOpMode {
    public WebcamBot robot = new WebcamBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.camera.start();

        while (!opModeIsActive()) {
            dash.create(robot.detector.getValues()[0]);
            dash.create(robot.detector.getValues()[1]);
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            dash.create(robot.detector.getValues()[0]);
            dash.create(robot.detector.getValues()[1]);
            dash.update();
        }
    }

    @Override
    public void stopLinearOpMode() {
        robot.camera.stop();
    }
}