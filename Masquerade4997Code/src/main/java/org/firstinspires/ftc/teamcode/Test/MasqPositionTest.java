package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.TestBot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "PositionTest", group = "T")
public class MasqPositionTest extends MasqLinearOpMode implements Constants {
    private TestBot thanos = new TestBot();
    public void runLinearOpMode() throws InterruptedException {
        thanos.mapHardware(hardwareMap);
        thanos.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("Left Inches: ", thanos.tracker.getLeftInches());
            dash.create("Right Inches: ", thanos.tracker.getRightInches());
            dash.create("Y Inches: ", thanos.tracker.getRawYInches());
            dash.create("Heading: ", thanos.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            thanos.NFS(controller1);
            dash.create("Left1 Inches: ", thanos.driveTrain.leftDrive.motor1.getEncoder().getInches());
            dash.create("Left Inches: ", thanos.tracker.getLeftInches());
            dash.create("Right Inches: ", thanos.tracker.getRightInches());
            dash.create("Y Inches: ", thanos.tracker.getRawYInches());
            dash.create("Heading: ", thanos.tracker.getHeading());
            dash.create("X: ", thanos.tracker.getGlobalX());
            dash.create("Y: ", thanos.tracker.getGlobalY());
            thanos.tracker.updateSystem();
            dash.update();
        }
    }
}