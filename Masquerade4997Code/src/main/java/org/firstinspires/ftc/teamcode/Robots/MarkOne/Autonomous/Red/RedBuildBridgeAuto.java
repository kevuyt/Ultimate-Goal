package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "RedBuildBridgeAuto", group = "Prototype")
public class RedBuildBridgeAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.driveTrain.setClosedLoop(true);
        robot.initializeAutonomous();
        robot.lift.setClosedLoop(true);
        robot.lift.setKp(0.001);

        robot.resetServos();
        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        robot.foundationHook.raise();
        sleep();
        robot.strafe(35, 90, 1.5);
        robot.turnAbsolute(0);
        robot.drive(35, 0.25);
        robot.foundationHook.lower();
        sleep();
        robot.drive(50, 0.25 ,Direction.BACKWARD,3);
        robot.foundationHook.raise();
        sleep();
        robot.drive(2,Direction.BACKWARD);
        robot.strafe(60, -90, 2);
        robot.foundationHook.mid();
        sleep();
        robot.turnAbsolute(0);
        robot.drive(33);
        robot.strafe(45, -90,2);
        robot.turnAbsolute(0);
    }
}
