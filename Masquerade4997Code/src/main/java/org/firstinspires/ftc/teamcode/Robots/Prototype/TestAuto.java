package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/3/2019
 */
@Autonomous(name = "TestAuto", group = "Prototype")
public class TestAuto extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.driveTrain.setClosedLoop(true);

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        robot.driveAbsoluteAngle(80,0);
    }
}
