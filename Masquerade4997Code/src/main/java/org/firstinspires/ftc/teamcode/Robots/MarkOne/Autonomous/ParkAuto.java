package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 1/18/2020
 */
@Autonomous(name = "Park", group = "MarkOne")
public class ParkAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        robot.gotoXY(new MasqPoint(24,0,0));
    }
}
