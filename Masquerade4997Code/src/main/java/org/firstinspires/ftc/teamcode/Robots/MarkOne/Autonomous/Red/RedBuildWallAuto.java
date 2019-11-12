package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "RedBuildWallAuto", group = "Prototype")
public class RedBuildWallAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.driveTrain.setClosedLoop(true);
        robot.lift.setClosedLoop(true);
        robot.lift.setKp(0.001);

        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        robot.foundationHook.raise();
        sleep();
        robot.strafe(30, 90, 1.5);
        robot.turnAbsolute(0);
        robot.drive(35, 0.25);
        robot.foundationHook.lower();
        sleep();
        robot.drive(50, 0.25 ,Direction.BACKWARD,3);
        robot.foundationHook.raise();
        sleep();
        robot.drive(5,Direction.BACKWARD);
        robot.strafe(50, -90, 2);
        robot.foundationHook.mid();
        sleep();
        robot.turnAbsolute(0);
        robot.drive(3,Direction.BACKWARD);
        robot.strafe(35, -90,2);
        robot.turnAbsolute(0);
        robot.drive(1);

    }
}
