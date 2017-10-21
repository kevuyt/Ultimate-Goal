package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "JewelAuto", group = "Autonomus")
public class JewelAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            //dash.create(robot.imu);
            dash.create(robot.jewelColor);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        while (opModeIsActive()) {
            if (robot.jewelColor.isBlue()) {
                dash.create("BLUE");
            } else dash.create("RED");
            dash.update();
        }
    }
}