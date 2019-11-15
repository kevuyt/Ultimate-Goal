package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/1/2019
 */

@TeleOp(name = "CameraTestTeleop", group = "Prototype")
public class CameraTestTeleop extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        try {
            robot.detector.skystoneDetector.setCropSettings(0, 0, 0, 0);
        }catch (Exception e) {
            e.printStackTrace();
        }
        robot.detector.start();
        while(!opModeIsActive()) {
                dash.create("Stone Detected: ", robot.detector.isDetected());
                dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            if (robot.detector.isDetected()) dash.create(robot.detector.getStoneSkystonePosition());
            else dash.create("Not Detected");
            dash.update();
        }
    }
}
