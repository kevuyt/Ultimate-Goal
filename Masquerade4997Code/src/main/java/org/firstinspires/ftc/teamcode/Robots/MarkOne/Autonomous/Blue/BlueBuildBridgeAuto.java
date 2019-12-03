package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.Strafe;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "BlueBuild", group = "MarkOne")
public class BlueBuildBridgeAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        robot.foundationHook.raise();
        robot.blockPusher.setPosition(1);

        robot.strafe(40, Strafe.LEFT, 1.5);
        robot.drive(42, 0.25);
        robot.turnAbsolute(0);
        robot.foundationHook.lower();
        sleep(1);
        robot.drive(55, 0.25 ,Direction.BACKWARD,3);
        robot.foundationHook.raise();
        sleep();

        robot.strafe(60, Strafe.RIGHT, 2);
        robot.turnAbsolute(0);
        robot.foundationHook.mid();
        robot.drive(15, 0.25);
        robot.turnAbsolute(90);
        robot.drive(15, 0.25);
    }
}