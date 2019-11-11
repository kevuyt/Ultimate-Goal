package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "BlueBuildWallAuto", group = "Prototype")
public class BlueBuildWallAuto extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

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

        robot.raiseFoundationHook();
        robot.blockPusher.setPosition(1);
        robot.strafe(25, Direction.LEFT, 1.5);
        robot.drive(40, 0.25);
        robot.lowerFoundationHook();
        sleep(1);
        robot.drive(50, 0.25 ,Direction.BACKWARD,3);
        robot.raiseFoundationHook();
        sleep();
        robot.strafe(70, Direction.RIGHT, 2);
        robot.midFoundationHook();
        robot.turnAbsolute(0);
        robot.drive(1.5, Direction.BACKWARD);
        sleep();
        robot.strafe(30, Direction.RIGHT,2);
        robot.turnAbsolute(0);
    }
}
