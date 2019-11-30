package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 10/27/2019
 */
@Autonomous(name = "SkystoneDetectionTest", group = "Prototype")
@Disabled
public class SkystoneDetectionTest extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.detector.start();

        //SkystoneDetector.SkystonePosition skystonePosition;
        while (!opModeIsActive()) {
            dash.create("Stone Detected: ", robot.detector.isDetected());
        }

        waitForStart();

        if (!robot.detector.isDetected()) robot.runStoneLeft(hardwareMap);
        else {
            /*skystonePosition = robot.detector.skystoneDetector.getSkystonePosition();
            if (skystonePosition == null) robot.runStoneLeft(hardwareMap);
            else if (skystonePosition == SkystoneDetector.SkystonePosition.LEFT) robot.runStoneLeft(hardwareMap);
            else if (skystonePosition == SkystoneDetector.SkystonePosition.RIGHT) robot.runStoneRight(hardwareMap);
            else if (skystonePosition == SkystoneDetector.SkystonePosition.MIDDLE) robot.runStoneMiddle(hardwareMap);
            else robot.runStoneLeft(hardwareMap);*/
        }
    }
}
