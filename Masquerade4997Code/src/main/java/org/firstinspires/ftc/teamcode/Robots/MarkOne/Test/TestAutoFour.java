package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-02.
 * Project: MasqLib
 */
@Autonomous(name = "gotoXYTank Test" , group = "MarkOne")
public class TestAutoFour extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while (!opModeIsActive()) {
            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            robot.tracker.updateSystem();
            dash.update();
        }

        waitForStart();
        robot.gotoXYTank(30,30, 5, 120, 1);
    }
}
