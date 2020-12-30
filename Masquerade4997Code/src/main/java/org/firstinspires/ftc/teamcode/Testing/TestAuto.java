package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/28/2020
 */

@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends MasqLinearOpMode {
    private TestBot robot = new TestBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(!opModeIsActive()) {
            robot.tracker.updateSystem();

            dash.create("X: "+ robot.tracker.getGlobalX());
            dash.create("Y: "+ robot.tracker.getGlobalY());
            dash.update();
        }

        waitForStart();

        robot.shoot(1);
        robot.shoot(.75);
        robot.shoot(0.5);
        robot.shoot(0.25);
        robot.shoot(-0.25);
        robot.shoot(-0.5);
        robot.shoot(-0.75);
        robot.shoot(-1);
    }
}