package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 1/18/2020
 */
@Autonomous(name = "Park", group = "MarkOne")
public class ParkAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private int sleeptime;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Use Y on controller 1 to increase sleep time");
            dash.create("Use A on controller 1 to decrease sleep time");
            dash.create("Max sleep time is 25 sec");
            dash.create("Sorry if it take multiple presses to register a change :(");
            if (controller1.yOnPress()) sleeptime++;
            if (controller1.aOnPress()) sleeptime--;
            sleeptime = Range.clip(sleeptime, 0, 25);
            dash.create("sleep", sleeptime);
            controller1.update();
            dash.update();
        }

        waitForStart();
        sleep(sleeptime);
        robot.xyPath(2,new MasqWayPoint().setPoint(0,24,0).setDriveCorrectionSpeed(0.02));
    }
}
