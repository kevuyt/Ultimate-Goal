package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "PositionTest", group = "T")
@Disabled
public class MasqPositionTest extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.tracker.reset();
        while (!opModeIsActive()) {
            dash.create("Left Inches: ", falcon.tracker.getLeftInches());
            dash.create("Right Inches: ", falcon.tracker.getRightInches());
            dash.create("Y Inches: ", falcon.tracker.getRawYInches());
            dash.create("Heading: ", falcon.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        falcon.drive(30, 0.8);
    }
}
