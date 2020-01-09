package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/3/2019
 */
@Autonomous(name = "(-20, -20, 90) --> (0...)", group = "MarkOne")
@Disabled
public class TestAutoOne extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while (!opModeIsActive()) {
            dash.create(robot.tracker);
            robot.tracker.updateSystem();
            dash.update();
        }

        waitForStart();

        robot.gotoXY(new MasqPoint(-20,-20, 90),30);
        robot.gotoXY(new MasqPoint(0,0, 0),30);
    }
}