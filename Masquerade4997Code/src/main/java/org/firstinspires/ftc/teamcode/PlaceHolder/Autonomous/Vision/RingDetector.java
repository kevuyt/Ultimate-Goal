package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import Library4997.MasqCV.detectors.MasqCVDetector;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class RingDetector extends MasqCVDetector {
    //private MasqCVColorFilter yellowFilter = new LeviColorFilter(YELLOW, 10);
    //private double averagestd;
    private Mat region_Cb;
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private int avg;
    @Override
    public Mat process(Mat input) {
        //input = cropMat(input,tl, br);

        //workingMat = input.clone();
        //displayMat = input.clone();

        /*List<MatOfPoint> contoursYellow = findContours(yellowFilter, workingMat);
        List<Rect> rects = contoursToRects(contoursYellow);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rects, 5);
        foundRect = chooseBestRect(listOfBlobs);

        drawContours(contoursYellow, new Scalar(0, 255, 0));
        List<MatOfPoint> contoursYellow = findContours(yellowFilter, workingMat);*/


        //found = foundRect.area() > minimumArea;

        /*if (found) {
            drawRect(foundRect, new Scalar(0, 255, 0));
            Imgproc.putText(displayMat, "Rings", foundRect.tl(),0,1,new Scalar(255,255,255));
        }

        return displayMat;*/

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
        region_Cb = Cb.submat(new Rect(tl, br));
        avg = (int) Core.mean(region_Cb).val[0];
        Imgproc.rectangle(input, tl, br, new Scalar(0,0,255), 2);
        return input;
    }
    public int getAverage() {
        return avg;
    }

}
