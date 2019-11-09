package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "BlueBuildStoneAuto", group = "Prototype")
public class BlueBuildStoneAuto extends MasqLinearOpMode {
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
        robot.strafe(40, Direction.LEFT, 1.5);
        robot.drive(42, 0.25);
        robot.turnAbsolute(10,Direction.LEFT );
        robot.lowerFoundationHook();
        sleep(1);
        robot.drive(50, 0.25 ,Direction.BACKWARD,3);
        robot.raiseFoundationHook();
        sleep();
        robot.strafe(65, Direction.RIGHT, 2);
        robot.turnAbsolute(5, Direction.RIGHT);
        robot.midFoundationHook();
        robot.drive(23.5);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.drive(30);
        robot.turnAbsolute(45,Direction.LEFT);
        robot.intake.setVelocity(1);
        robot.drive(15, 0.5);
        sleep(.1);
        robot.intake.setVelocity(0);
        robot.drive(15, Direction.BACKWARD);
        robot.turnAbsolute(135,Direction.LEFT);
        robot.drive(45);
        sleep();
        robot.intake.setVelocity(-1);
        robot.drive(30, Direction.BACKWARD);
        robot.strafe(5, Direction.RIGHT);



    }
}
