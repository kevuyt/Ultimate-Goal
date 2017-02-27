package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "DEBUG", group = "G3") // change name
public class DEBUG_AUTO extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;
        MasqRobot chimera = new MasqRobot(telemetry);
        while (!isStarted()) {
            chimera.colorRejection.setActiveMode();
            chimera.runSensorTelemetry();
            telemetry.update();
            idle();
        }

        waitForStart();
    }
}