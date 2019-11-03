package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "BlueBuildAuto", group = "Prototype")
public class BlueBuildAuto extends MasqLinearOpMode {
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
        robot.strafe(2.6, -90, 1);
        robot.drive(39);
        robot.turnAbsolute(5, Direction.LEFT);
        robot.lowerFoundationHook();
        sleep();
        robot.drive(57, 0.25 ,Direction.BACKWARD,3);
        robot.raiseFoundationHook();
        robot.drive(5, 0.25,Direction.BACKWARD, 1);
        robot.strafe(150, 90, 5);

    }
}
