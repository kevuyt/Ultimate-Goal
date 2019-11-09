package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "RedBuildBridgeAuto", group = "Prototype")
public class RedBuildBridgeAuto extends MasqLinearOpMode {
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
        sleep();
        robot.strafe(30, 90, 1.5);
        robot.turnAbsolute(5, Direction.RIGHT);
        robot.drive(35, 0.25);
        robot.lowerFoundationHook();
        sleep();
        robot.drive(50, 0.25 ,Direction.BACKWARD,3);
        robot.raiseFoundationHook();
        sleep();
        robot.drive(2,Direction.BACKWARD);
        robot.strafe(52, -90, 2);
        robot.midFoundationHook();
        sleep();
        robot.turnAbsolute(5, Direction.LEFT);
        robot.drive(30);
        robot.strafe(39, -90,2);
        robot.turnAbsolute(5, Direction.LEFT);

    }
}
