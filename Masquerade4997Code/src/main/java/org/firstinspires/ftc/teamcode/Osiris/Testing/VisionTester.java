package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.*;
import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.TargetZone;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;
import org.opencv.core.Rect;

import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqUtils.getCenterPoint;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.ZoneFinder.findZone;

/**
 * Created by Keval Kataria on 3/7/2021
 */
@TeleOp(name = "VisionTester", group = "Test")
public class VisionTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    RingDetector ringDetector = new RingDetector();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);

        while(!opModeIsActive()) {
            TargetZone zone = ringDetector.findZone();

            dash.create("Remove download wire");
            dash.create("Zone: " + zone);
            dash.create("Control: " + ringDetector.getControl());
            dash.create("Top: " + ringDetector.getTop());
            dash.create("Bottom: " + ringDetector.getBottom());
            dash.update();

            if(isStopRequested()) {
                robot.camera.stop();
                break;
            }
        }

        waitForStart();

        ringDetector.switchDetection();

        while(opModeIsActive()) {
            Rect rect = ringDetector.getFoundRect();

            dash.create("Center Point X: ", getCenterPoint(rect).x);
            dash.create("Height: ", rect.height);
            dash.update();
        }

        robot.camera.stop();
    }
}