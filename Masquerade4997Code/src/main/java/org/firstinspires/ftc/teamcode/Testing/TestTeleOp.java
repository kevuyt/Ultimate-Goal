package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends MasqLinearOpMode {
    private TestBot robot = new TestBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            robot.tracker.updateSystem();
            robot.driveTrain.setClosedLoop(false);
            dash.create("X: "+ robot.tracker.getGlobalX());
            dash.create("Y: "+ robot.tracker.getGlobalY());
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.driveTrain.setClosedLoop(true);
            robot.MECH(controller1);

            robot.tracker.updateSystem();

            dash.create("X: "+ robot.tracker.getGlobalX());
            dash.create("Y: "+ robot.tracker.getGlobalY());
            dash.create("X Inches: " + robot.intake.getInches());
            dash.create("yL Inches: " + robot.encoder1.getInches());
            dash.create("yR Inches: " + robot.encoder2.getInches());
            dash.update();
        }
    }
}