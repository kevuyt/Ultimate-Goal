package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SkystoneDetection.DogeDetector;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 10/27/2019
 */
@Autonomous(name = "SkystoneDetectionAutonomous", group = "Prototype")
@Disabled
public class SkystoneDetectionAutonomous extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.detector.start();

        DogeDetector.SkystonePosition skystonePosition;
        while (!opModeIsActive()) {
            dash.create("Stone Detected: ", robot.detector.isDetected());
        }

        waitForStart();

        if (!robot.detector.isDetected()) robot.runStoneLeft(hardwareMap);
        else {
            skystonePosition = robot.detector.getStoneSkystonePosition();

            if (skystonePosition == null) robot.runStoneLeft(hardwareMap);
            else if (skystonePosition == DogeDetector.SkystonePosition.LEFT) robot.runStoneLeft(hardwareMap);
            else if (skystonePosition == DogeDetector.SkystonePosition.RIGHT) robot.runStoneRight(hardwareMap);
            else if (skystonePosition == DogeDetector.SkystonePosition.MIDDLE) robot.runStoneMiddle(hardwareMap);
            else robot.runStoneLeft(hardwareMap);
        }
    }
}
