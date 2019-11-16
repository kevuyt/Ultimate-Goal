package org.firstinspires.ftc.teamcode.Robots.WebcamBot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SkystoneDetection.DogeDetector;

import Library4997.MasqRobot;

/**
 * Created by Keval Kataria on 11/16/2019
 */
public class WebcamBot extends MasqRobot {
    DogeDetector detector;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        detector = new DogeDetector(DogeDetector.Cam.WEBCAM,hardwareMap);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
    }
}
