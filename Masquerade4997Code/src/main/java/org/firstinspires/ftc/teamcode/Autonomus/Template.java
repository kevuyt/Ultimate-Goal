package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqWrappers.Direction;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */

@Autonomous(name = "Template-Auto", group = "Template")
public class Template extends LinearOpMode implements Constants {
    MasqTankDrive driveTrain = new MasqTankDrive();
    public void runOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            telemetry.addLine("Hello");
            telemetry.update();
        }
        waitForStart();
        //robot.sleep(robot.getDelay());
        driveTrain.setDistance(60);
        driveTrain.runToPosition(Direction.FORWARD, 0.7, 5);
    }
}
