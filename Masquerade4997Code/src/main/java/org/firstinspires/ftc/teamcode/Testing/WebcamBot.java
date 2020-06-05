package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqCV.MasqCamera;
import Library4997.MasqRobot;

import static Library4997.MasqCV.MasqCamera.Cam.WEBCAM;

/**
 * Created by Keval Kataria on 6/4/2020
 */
public class WebcamBot extends MasqRobot {
    TestDetector detector;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException{
        detector = new TestDetector();
        MasqCamera camera = new MasqCamera(detector, WEBCAM, hardwareMap);
        camera.start();
    }

    @Override
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        mapHardware(hardwareMap);
    }
}
