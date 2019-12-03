package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.DashBoard;
import MasqCV.detectors.skystone.SkystoneDetector;

/**
 * Created by Keval Kataria on 12/1/2019
 */
public class MarkOneDetector {
    private OpenCvWebcam webcam;
    private SkystoneDetector skystoneDetector;
    private int offset = 0;

    public MarkOneDetector(HardwareMap hwMap){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        skystoneDetector = new SkystoneDetector();
        webcam.setPipeline(skystoneDetector);
    }

    public void start(){
        webcam.openCameraDevice();
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stop(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public void pauseViewPort(){
        webcam.pauseViewport();
    }

    public void resumeViewPort(){
        webcam.resumeViewport();
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
    public int getImageWidth() {
        return skystoneDetector.getImageWidth();
    }
    public void setClippingMargins(int top, int left, int bottom, int right) {
        skystoneDetector.setClippingMargins(top, left, bottom, right);
        offset = left;
    }
    public void setClippingMargins(Point tl, Point br) {
        skystoneDetector.setClippingMargins(tl, br);
        offset = (int) tl.x;
    }
    public enum SkystonePosition {
        LEFT,MIDDLE,RIGHT
    }
    public SkystonePosition getPosition() {
        if ((MasqUtils.getCenterPoint(getFoundRect()).x - offset) < getImageWidth()/3) return SkystonePosition.LEFT;
        else if ((MasqUtils.getCenterPoint(getFoundRect()).x - offset) < (2 * getImageWidth()/3)) return SkystonePosition.MIDDLE;
        else return SkystonePosition.RIGHT;
    }
}
