package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 4/23/18.
 */
@Autonomous(name = "Sample", group = "Autonomus")
public class Sample extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(creed.positionTracker.imu);
            dash.update();
        }
        waitForStart();
        creed.sleep(robot.getDelay());
    }
}