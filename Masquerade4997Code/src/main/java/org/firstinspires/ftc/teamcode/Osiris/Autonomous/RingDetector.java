package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import org.opencv.core.*;

import MasqLibrary.MasqMath.MasqVector;
import MasqLibrary.MasqVision.*;

import static java.lang.Math.*;
import static java.lang.System.nanoTime;
import static org.opencv.core.Core.*;
import static org.opencv.imgproc.Imgproc.*;

/**
 * Created by Keval Kataria on 6/1/2020
 */

public class RingDetector extends MasqCVDetector {
    double top, control, bottom;
    double prevTime = 0;

    @Override
    public Mat process(Mat input) {
        double time = nanoTime();

        if(time - prevTime > 1e9) {
            prevTime = time;
            workingMat = input.clone();
            displayMat = input.clone();

            cvtColor(workingMat, workingMat, COLOR_RGB2YCrCb);
            extractChannel(workingMat, workingMat, 1);

            Rect topRect = new Rect(tl, new Point(br.x, tl.y + (br.y - tl.y) * 3.0 / 4));
            Rect bottomRect = new Rect(new Point(tl.x, topRect.br().y), br);
            Rect controlRect = new Rect(new Point(tl.x, br.y), new Point(br.x, br.y +
                    topRect.height + bottomRect.height));

            control = mean(workingMat.clone().submat(controlRect)).val[0];
            top = mean(workingMat.clone().submat(topRect)).val[0];
            bottom = mean(workingMat.clone().submat(bottomRect)).val[0];

            workingMat.release();

            drawRect(controlRect, new Scalar(255, 0, 0), false);
            drawRect(topRect, new Scalar(0, 0, 255), false);
            drawRect(bottomRect, new Scalar(0, 255, 0), false);
        }

        return displayMat;
    }

    public double getTop() {return top;}
    public double getBottom() {return bottom;}
    public double getControl() {return control;}

    public enum TargetZone {A,B,C}

    public TargetZone findZone () {
        if (abs(getTop()- getBottom()) > 10) return TargetZone.B;
        else if (abs(((getTop() + getBottom()) / 2 - getControl())) > 10) return TargetZone.C;
        else return TargetZone.A;
    }
}