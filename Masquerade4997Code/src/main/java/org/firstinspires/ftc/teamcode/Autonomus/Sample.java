package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 4/23/18.
 */
@Autonomous(name = "Sample", group = "Autonomus")
public class Sample extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.positionTracker.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
    }
}