package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.HeightFinder;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 6/4/2020
 */
@TeleOp(name = "WebcamTeleop", group = "Test")
public class WebcamTeleop extends MasqLinearOpMode {
    private WebcamBot robot = new WebcamBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(!opModeIsActive()) {
            dash.create("Average: ", robot.detector.getAverage());
            dash.create("Zone: ", HeightFinder.findZone(robot.detector));
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            dash.create("Average: ", robot.detector.getAverage());
            dash.create("Zone: ", HeightFinder.findZone(robot.detector));
            dash.update();
        }
    }
}
