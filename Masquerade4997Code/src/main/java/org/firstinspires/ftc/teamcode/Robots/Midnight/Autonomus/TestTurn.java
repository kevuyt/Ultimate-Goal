package org.firstinspires.ftc.teamcode.Robots.Midnight.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@Autonomous(name = "TestTurn", group = "Testbot")
@Disabled

public class TestTurn extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();
        robot.turnAbsolute(90);
    }
}