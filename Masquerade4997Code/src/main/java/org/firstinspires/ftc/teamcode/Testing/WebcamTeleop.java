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
        try {

            robot.camera.start();
            while (!opModeIsActive()) {
                dash.create(robot.detector.getFoundRect().height);
                dash.update();
            }

        } catch(Exception e) {
            while(true) {
                dash.create(e);
                dash.update();
                sleep(100);
            }
        }

        waitForStart();

        while (opModeIsActive()) {
            dash.create(robot.detector.getFoundRect().height);
            dash.update();
        }
    }
}