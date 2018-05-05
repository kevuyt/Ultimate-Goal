package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTesterC1: Stall Detection Stop", group = "T")
public class MasqGeneralTesterC1 extends MasqLinearOpMode implements Constants {
    String direction = "Straight";
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("                      ________________    ");
            dash.create("                     /                \\  ");
            dash.create("                    /                  \\ ");
            dash.create("                   /                    \\ ");
            dash.create("                   \\                    / ");
            dash.create("                    \\                  / ");
            dash.create("                     \\                / ");
            dash.create("                      ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   ");
            dash.create("                             \\|/           ");
            dash.create("                         |__|__|__|      ");
            if (controller1.y()) direction = "Straight";
            else if (controller1.x()) direction ="Left";
            else if (controller1.b()) direction = "Right";
            dash.create(direction);
            dash.update();
        }
        waitForStart();
        robot.go(5, 0, 0);
        robot.go(0, 0, 0);
    }
}