package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import Library4997.MasqCV.detectors.MasqCVDetector;

/**
 * Created by Keval Kataria on 6/1/2020
 */
public class RingDetector extends MasqCVDetector {
    double avg, control;

    @Override
    public Mat processFrame(Mat input) {
        workingMat = input.clone();
        displayMat = input.clone();

        Imgproc.cvtColor(workingMat, workingMat,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(workingMat,workingMat,1);

        Rect workingRect = new Rect(tl,br);
        Rect controlRect = new Rect(new Point(tl.x,br.y),new Point(br.x,br.y+workingRect.height));

        control = Core.mean(workingMat.clone().submat(controlRect)).val[0];
        avg = Core.mean(workingMat.submat(workingRect)).val[0];

        drawRect(controlRect,new Scalar(255,0,0),false);
        drawRect(workingRect,new Scalar(0,0,255),false);

        return displayMat;
    }
    public double getAverage() {return avg;}
    public double getControl() {return control;}
}