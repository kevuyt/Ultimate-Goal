package org.firstinspires.ftc.teamcode.SkystoneDetection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import Library4997.MasqWrappers.DashBoard;
import MasqCV.detectors.skystone.SkystoneDetector;
/**
 * Created by Keval Kataria on 10/27/2019
 */
public class DogeDetector {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webcam;
    public SkystoneDetector skystoneDetector;
    private Cam cam;

    public enum Cam{
        PHONE, WEBCAM
    }

    public DogeDetector(Cam cam, HardwareMap hwMap){
        this.cam = cam;
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        if(cam.equals(Cam.PHONE)){
            phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            phoneCamera.setPipeline(skystoneDetector);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam = new OpenCvWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            webcam.setPipeline(skystoneDetector);
        }
    }

    public void start(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.openCameraDevice();
            webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        }
    }

    public void stop(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pauseViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.pauseViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.pauseViewport();
    }

    public void resumeViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.resumeViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.resumeViewport();
    }

    public Rect getFoundRect() {
        return skystoneDetector.foundRectangle();
    }
    public boolean isDetected(){
        return skystoneDetector.isDetected();
    }
    public Mat cropMat(Mat input, Point tl, Point br)  {
        if (!(tl == null || br == null || tl.x >= input.width() || tl.y >= input.height() || tl.x < 0 || tl.y < 0 || br.x > input.width() || br.y > input.height() || br.x <= 0 || br.y <= 0)) {
            Imgproc.rectangle(input,new Point(tl.x, tl.y),new Point(br.x, br.y),new Scalar(0), -1);
        }
        else {
            DashBoard.getDash().create("Cropping failed due to invalid cropping margins");
        }
        return input;
    }
}