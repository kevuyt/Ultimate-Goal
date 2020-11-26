package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp
public class TestTeleop extends MasqLinearOpMode {
    public TestBot robot = new TestBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while (!opModeIsActive()) {
            robot.tracker.updateSystem();
            dash.create("X: "+ robot.tracker.getGlobalX());
            dash.create("Y: "+ robot.tracker.getGlobalY());
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.MECH(controller1);
            robot.tracker.updateSystem();
            dash.create("X: "+ robot.tracker.getGlobalX());
            dash.create("Y: "+ robot.tracker.getGlobalY());
            dash.update();
        }
    }
}