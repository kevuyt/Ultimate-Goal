package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/1/2019
 */

@TeleOp(name = "CameraTestTeleop", group = "Prototype")
public class CameraTestTeleop extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

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
