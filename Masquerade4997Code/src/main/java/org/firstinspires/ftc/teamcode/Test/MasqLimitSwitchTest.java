package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/29/17.
 */
@Autonomous(name = "Limit Switch Test", group = "Autonomus")
public class MasqLimitSwitchTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()){
            dash.create(robot.jewelColor);
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {

        }
    }
    @Override
    public void stopLinearOpMode(){

    }
}
