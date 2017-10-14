package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "JewelAuto", group = "Autonomus")
public class JewelAuto extends MasqLinearOpMode implements Constants {
    public void run() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.jewelArm.setPower(1);
    }
}