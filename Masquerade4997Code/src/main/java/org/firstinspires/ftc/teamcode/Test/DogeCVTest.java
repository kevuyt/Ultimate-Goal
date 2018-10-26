package org.firstinspires.ftc.teamcode.Test;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/26/18.
 * Project: MasqLib
 */

@Autonomous(name = "Doge", group = "T")
public class DogeCVTest extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        falcon.goldAlignDetector.useDefaults();

        // Optional Tuning
        falcon.goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        falcon.goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        falcon.goldAlignDetector.downscale = 0.4; // How much to downscale the input frames

        falcon.goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //falcon.goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        falcon.goldAlignDetector.maxAreaScorer.weight = 0.005;

        falcon.goldAlignDetector.ratioScorer.weight = 5;
        falcon.goldAlignDetector.ratioScorer.perfectRatio = 1.0;
        falcon.goldAlignDetector.enable();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            dash.create("X:", falcon.goldAlignDetector.getXPosition());
            dash.update();
        }
        falcon.goldAlignDetector.disable();
    }
}