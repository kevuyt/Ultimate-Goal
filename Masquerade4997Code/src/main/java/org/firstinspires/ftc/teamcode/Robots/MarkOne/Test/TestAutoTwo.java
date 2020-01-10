package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqPositionTracker;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-02.
 * Project: MasqLib
 */
@Autonomous(name = "(0, 20, 0)", group = "MarkOne")
@Disabled
public class TestAutoTwo extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while (!opModeIsActive()) {
            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            robot.tracker.updateSystem();
            dash.update();
        }

        waitForStart();

        robot.gotoXY(new MasqPoint(0,20, 0),30);
    }
}
