package org.firstinspires.ftc.teamcode.Robots.WebcamBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/29/2019
 */
@TeleOp(name = "Cam")
public class WebcamTeleop2 extends MasqLinearOpMode {
    private WebcamBot robot = new WebcamBot();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.detector.start();
        robot.detector.skystoneDetector.setClippingMargins(50,0,100,0);
        while(!opModeIsActive()) {
            dash.create("Detected", robot.detector.isDetected());
            dash.update();
        }
        waitForStart();

        while(opModeIsActive()) {
            dash.create("Left: ", robot.detector.getFoundRect().tl().x);
            dash.create("Top: ",robot.detector.getFoundRect().tl().y);
            dash.create("Right: ", robot.detector.getFoundRect().br().x);
            dash.create("Bottom: ", robot.detector.getFoundRect().br().y);
            dash.update();
        }
    }
}