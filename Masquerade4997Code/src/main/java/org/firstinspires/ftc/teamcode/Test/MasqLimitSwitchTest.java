package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/29/17.
 */
@Autonomous(name = "Color Sensor Test", group = "Autonomus")
@Disabled
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
