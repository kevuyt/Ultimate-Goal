package org.firstinspires.ftc.teamcode.Test;

/**
 * Created by Archishmaan Peyyety on 8/25/18.
 * Project: MasqLib
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqMotorTesting", group = "T")
@Disabled
public class MasqGoTest extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(falcon.tracker.getGlobalX());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            dash.create("Position: ", falcon.driveTrain.leftDrive.getAbsolutePosition());
            dash.create("Velocity: ", falcon.driveTrain.leftDrive.getVelocity());
            dash.create("Acceleration: ", falcon.driveTrain.leftDrive.getAcceleration());
            falcon.driveTrain.leftDrive.setPower(0.7);
        }
    }
}