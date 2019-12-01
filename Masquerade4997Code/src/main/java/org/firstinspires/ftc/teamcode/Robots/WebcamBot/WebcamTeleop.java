package org.firstinspires.ftc.teamcode.Robots.WebcamBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/29/2019
 */
@TeleOp(name = "Cam")
public class WebcamTeleop extends MasqLinearOpMode {
    private WebcamBot robot = new WebcamBot();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.detector.start();
        robot.detector.skystoneDetector.setClippingMargins(100,80,110,70);
        while(!opModeIsActive()) {
            dash.create("Detected", robot.detector.isDetected());
            dash.update();
        }
        waitForStart();

        while(opModeIsActive()) {
            dash.create("Left: ", robot.detector.getFoundRect().tl().x);
            dash.create("Right: ", robot.detector.getFoundRect().br().x);
            dash.create("Center x: ", MasqUtils.getCenterPoint(robot.detector.getFoundRect()).x);
            dash.create("Width: ", robot.detector.skystoneDetector.getImageWidth());
            dash.create("Area: ", robot.detector.getFoundRect().area());
            if ((MasqUtils.getCenterPoint(robot.detector.getFoundRect()).x - 80) < robot.detector.skystoneDetector.getImageWidth()/3) dash.create("Position: LEFT");
            else if ((MasqUtils.getCenterPoint(robot.detector.getFoundRect()).x - 80) < (2 * robot.detector.skystoneDetector.getImageWidth()/3)) dash.create("Position: MIDDLE");
            else dash.create("Position: RIGHT");
            dash.update();
        }
    }
}