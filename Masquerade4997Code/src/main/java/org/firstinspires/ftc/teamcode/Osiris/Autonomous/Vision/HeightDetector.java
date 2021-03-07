package org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import Library4997.MasqVision.MasqCVDetector;

import static org.opencv.core.Core.*;
import static org.opencv.imgproc.Imgproc.*;

/**
 * Created by Keval Kataria on 6/1/2020
 */
public class HeightDetector extends MasqCVDetector {
    double top, control, bottom;

    @Override
    public Mat processFrame(Mat input) {
        workingMat = input.clone();
        displayMat = input.clone();

        cvtColor(workingMat, workingMat,COLOR_RGB2YCrCb);
        extractChannel(workingMat,workingMat,1);

        Rect topRect = new Rect(tl,new Point(br.x,tl.y + (br.y-tl.y) * 3.0 / 4));
        Rect bottomRect = new Rect(new Point(tl.x,topRect.br().y), br);
        Rect controlRect = new Rect(new Point(tl.x,br.y),new Point(br.x,br.y + topRect.height + bottomRect.height));
        control = mean(workingMat.clone().submat(controlRect)).val[0];
        top = mean(workingMat.submat(topRect)).val[0];
        bottom = mean(workingMat.submat(bottomRect)).val[0];

        workingMat.release();

        drawRect(controlRect,new Scalar(255,0,0),false);
        drawRect(topRect,new Scalar(0,0,255),false);
        drawRect(bottomRect,new Scalar(0,255,0),false);

        return displayMat;
    }
    public double getTop() {return top;}
    public double getBottom() {return bottom;}
    public double getControl() {return control;}
}