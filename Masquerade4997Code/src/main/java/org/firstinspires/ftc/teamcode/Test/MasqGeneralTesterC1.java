package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTesterC1: Stall Detection Stop", group = "T")
public class MasqGeneralTesterC1 extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    String direction = "Straight";
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            if (controller1.y()) direction = "Straight";
            else if (controller1.x()) direction ="Left";
            else if (controller1.b()) direction = "Right";
            dash.create(direction);
            dash.update();
        }
        waitForStart();
        creed.go(5, 0, 0);
        creed.go(0, 0, 0);
    }
}