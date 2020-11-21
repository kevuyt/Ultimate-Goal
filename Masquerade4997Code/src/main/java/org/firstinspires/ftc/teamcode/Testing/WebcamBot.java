package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.RingDetector;

import Library4997.MasqCV.MasqCamera;
import Library4997.MasqRobot;

import static Library4997.MasqCV.MasqCamera.Cam.WEBCAM;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPSIDE_DOWN;

/**
 * Created by Keval Kataria on 6/4/2020
 */
public class WebcamBot extends MasqRobot {
    RingDetector detector;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException{
        detector = new RingDetector();
        detector.setClippingMargins(125,90,90,200);
        MasqCamera camera = new MasqCamera(detector, WEBCAM, hardwareMap);
        camera.start(UPSIDE_DOWN);
    }

    @Override
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        mapHardware(hardwareMap);
    }
}
