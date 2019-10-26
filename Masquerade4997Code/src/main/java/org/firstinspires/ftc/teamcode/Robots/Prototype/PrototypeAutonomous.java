package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "PrototypeAutonomous", group = "Prototype")
public class PrototypeAutonomous extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();
    @Override
    public void runLinearOpMode() {
        while(!opModeIsActive()) {
            robot.mapHardware(hardwareMap);
        }

        waitForStart();

        robot.drive(37);
    }
}
