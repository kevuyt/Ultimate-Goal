package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MasqGeneralTesterC1", group = "T")
public class MasqGeneralTesterC1 extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        creed.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("Left Inches: ", creed.tracker.getLeftInches());
            dash.create("Right Inches: ", creed.tracker.getRightInches());
            dash.create("Y Inches: ", creed.tracker.getRelativeYInches());
            dash.create("Heading: ", creed.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            creed.MECH(controller1);
            dash.create("Left Inches: ", creed.tracker.getLeftInches());
            dash.create("Right Inches: ", creed.tracker.getRightInches());
            dash.create("Y Inches: ", creed.tracker.getRelativeYInches());
            dash.create("Heading: ", creed.tracker.getHeading());
            dash.create("X: ", creed.tracker.getGlobalX());
            dash.create("Y: ", creed.tracker.getGlobalY());
            creed.tracker.updateSystem();
            dash.update();
        }
    }
}