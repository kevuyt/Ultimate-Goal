package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "TestingAuto", group = "Prototype")
public class TestingAuto extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();


    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.driveTrain.setClosedLoop(true);
        robot.lift.setClosedLoop(true);
        robot.lift.setKp(0.001);

        robot.resetServos();
        while(!opModeIsActive()) {

            dash.create("Hello ");
            dash.update();
        }

        waitForStart();

        robot.strafe(30, 90);
        robot.drive(29);
        robot.lowerFoundationHook();
        robot.drive(29, Direction.BACKWARD);
        robot.raiseFoundationHook();
        robot.strafe(340, -90);

    }
}
