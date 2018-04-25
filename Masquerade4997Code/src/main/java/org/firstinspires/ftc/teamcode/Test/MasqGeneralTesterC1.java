package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTesterC1: Stall Detection Stop", group = "T")
public class MasqGeneralTesterC1 extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("BLUE_SENSOR_BLUE ", robot.blueLineDetector.getBlue());
            dash.create("BLUE_SENSOR_RED ", robot.blueLineDetector.getRed());
            dash.create("BLUE DETECTOR ", robot.blueLineDetector.isBlue());
            dash.create("RED_SENSOR_BLUE ", robot.redLineDetector.getBlue());
            dash.create("RED_SENSOR_RED ", robot.redLineDetector.getRed());
            dash.create("RED DETECTOR ", robot.redLineDetector.isRed());
            dash.update();
        }
        waitForStart();
        robot.go(5, 5, 0);
        robot.go(0, 0, 0);
    }
}