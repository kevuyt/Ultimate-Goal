package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.AUTO;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@Autonomous(name = "XYPathTester", group = "Test")
public class XYPathTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap, AUTO);

        dash.create("Initialized");
        dash.update();

        waitForStart();

        robot.xyPath(new MasqWayPoint(24,0,0).setTimeout(30));

        robot.tracker.updateOverTime(29-timeoutClock.seconds());
    }
}