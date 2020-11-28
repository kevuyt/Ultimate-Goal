package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
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

        robot.xyPath(2, new MasqWayPoint().setPoint(0,-24,90));
        sleep(20.0);
    }
}
