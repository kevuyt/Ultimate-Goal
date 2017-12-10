package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/29/17.
 */
@Autonomous(name = "LimitSwitch Test", group = "Autonomus")
public class MasqLimitSwitchTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()){
            dash.create(robot.bottomLimit.getState());
            dash.update();
            robot.sleep(10);
        }
    }
}
