package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector;
import org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector.TargetZone;
import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;
import org.opencv.core.Rect;

import MasqLibrary.MasqMath.MasqVector;
import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqResources.MasqUtils.getCenterPoint;
import static MasqLibrary.MasqRobot.OpMode.AUTO;

/**
 * Created by Keval Kataria on 3/7/2021
 */

@TeleOp(group = "Test")
public class VisionTester extends MasqLinearOpMode {
    private final Osiris robot = new Osiris();
    RingDetector detector;

    @Override
    public void runLinearOpMode() {
        robot.init(AUTO);
        detector = (RingDetector) robot.camera.detector;

        while(!opModeIsActive()) {
            TargetZone zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:", detector.getControl());
            dash.create("Top:", detector.getTop());
            dash.create("Bottom:", detector.getBottom());
            dash.update();

            if(isStopRequested()) {
                robot.camera.stop();
                break;
            }
        }

        waitForStart();

        while(opModeIsActive()) {
            TargetZone zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:", detector.getControl());
            dash.create("Top:", detector.getTop());
            dash.create("Bottom:", detector.getBottom());
            dash.update();
        }

        robot.camera.stop();
    }
}