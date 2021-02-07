package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        dash.create("Initialized");
        dash.update();

        waitForStart();

        robot.xyPath(10, new MasqWayPoint(24,0,0).setTimeout(10));

        robot.tracker.updateOverTime(29-timeoutClock.seconds());
    }
}