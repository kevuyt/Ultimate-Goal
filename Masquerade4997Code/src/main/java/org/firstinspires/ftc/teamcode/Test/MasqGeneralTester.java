package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTester", group = "T")
public class MasqGeneralTester extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("IMU ", robot.imu.getAbsoluteHeading());
            dash.update();
        }
        waitForStart();
        robot.turnAbsolute(90, Direction.LEFT);
        robot.turnAbsolute(0, Direction.BACKWARD);

    }
}