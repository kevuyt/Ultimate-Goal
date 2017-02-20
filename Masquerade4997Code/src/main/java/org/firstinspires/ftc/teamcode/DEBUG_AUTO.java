package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

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
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.colorRejection.setActiveMode();
            chimera.runSensorTelemetry();
            telemetry.update();
            idle();
        }

        waitForStart();
    }
}