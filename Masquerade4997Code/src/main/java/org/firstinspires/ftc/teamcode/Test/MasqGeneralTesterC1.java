package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTesterC1: Stall Detection Stop", group = "T")
public class MasqGeneralTesterC1 extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.intake.motor1.setStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(OUTAKE);
            }
        });
        robot.intake.motor1.setUnStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(INTAKE);
            }
        });
        robot.blueLineDetector.setActive();
        robot.redLineDetector.setActive();
        robot.blueLineDetector.setMargin(20);
        robot.redLineDetector.setMargin(20);
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
        robot.intake.motor1.enableStallDetection();
        robot.intake.setPower(INTAKE);
        robot.sleep(10000);
        int i = 0;
        while (i < 20) {
            robot.intake.motor1.setStallDetection(false);
            robot.intake.setPower(0);
            i++;
        }
        robot.sleep(10000);
        robot.intake.motor1.setStallDetection(true);
        robot.intake.setPower(INTAKE);
        robot.sleep(10000);
    }
}