package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "Template", group = "Test") // change name
@Disabled
public class Template extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }
    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.runAllTelemetry();
            telemetry.update();
            idle();
        }
        waitForStart();
    }
}