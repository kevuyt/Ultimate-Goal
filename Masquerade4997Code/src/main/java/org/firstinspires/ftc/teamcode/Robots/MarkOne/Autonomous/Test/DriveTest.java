package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-02-26.
 * Project: MasqLib
 */
@Autonomous(name = "DriveTest", group = "MarkOne")
public class DriveTest extends MasqLinearOpMode {
    public MarkOne robot = new MarkOne();
    private double hookPos;
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        while (!opModeIsActive()) {
            dash.create("Hello, this is a test for a mech based gotoXY.");
            dash.create(robot.tracker);
            robot.tracker.updateSystem();
            dash.update();
        }
        robot.drive(-20);
        waitForStart();

    }
}
