package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.MasqRobot.Direction;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "ShootParkBlue", group = "G2") // change name
@Disabled
public class ShootParkBlue extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
       // boolean telemetrizeModules;
        double Power_Optimal = 0.50;
        double POWER_LOW = 0.3;
        double HIGH_POWER = 0.90;
        MasqRobot chimera = new MasqRobot(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0.8);
            telemetry.addData("hue", chimera.leftColor.hue());
            chimera.runAllTelemetry();
            telemetry.update();
            idle();
        }
        waitForStart();
        chimera.drive(Power_Optimal, 15, Direction.BACKWARD);
        chimera.setPowerShooter(-0.8);
        chimera.sleep(3000);
        chimera.setPowerCollector(-1);
        chimera.setIndexer(0.2);
        chimera.sleep(500);
        chimera.setIndexer(0.8);
        chimera.sleep(3000);
        chimera.setPowerCollector(-1);
        chimera.setIndexer(0.2);
        chimera.sleep(3000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);
        chimera.drive(Power_Optimal, 130, Direction.BACKWARD);
        chimera.setPowerCollector(1);
        chimera.turn(Power_Optimal, 45, Direction.LEFT);
        chimera.drive(Power_Optimal, 200, Direction.FORWARD);
        chimera.setBrakeMode(10000);

    }
}