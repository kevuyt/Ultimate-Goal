package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 3/25/18.
 */
@Autonomous(name = "IntakeTest", group = "Autonomus")
public class IntakeTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.drive(60, POWER_OPTIMAL, Direction.BACKWARD);
    }
}