package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqSensors.MasqOpenCV;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/23/18.
 */
@Autonomous(name = "MasqOpenCVTest", group = "Autonomus")
public class MasqOpenCVTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.openCV.getCryptoboxDetector().setAlliance(MasqOpenCV.Alliance.RED);
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("CURRENT ROBOT STATUS: ");
            dash.create(robot.openCV.getCryptoboxDetector());
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
    }
}