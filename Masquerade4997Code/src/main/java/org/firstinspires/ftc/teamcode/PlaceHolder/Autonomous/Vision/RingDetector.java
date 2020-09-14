package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import Library4997.MasqCV.detectors.MasqCVDetector;
import Library4997.MasqCV.filters.LeviColorFilter;
import Library4997.MasqCV.filters.MasqCVColorFilter;

import static Library4997.MasqCV.filters.LeviColorFilter.ColorPreset.YELLOW;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class RingDetector extends MasqCVDetector {
    private MasqCVColorFilter yellowFilter = new LeviColorFilter(YELLOW, 50);

    @Override
    public Mat process(Mat input) {
        cropMat(input,tl, br);

        workingMat = input.clone();
        displayMat = input.clone();

        List<MatOfPoint> contoursYellow = findContours(yellowFilter, workingMat.clone());
        List<Rect> rects = contoursToRects(contoursYellow);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rects, 10);
        foundRect = chooseBestRect(listOfBlobs);

        drawContours(contoursYellow, new Scalar(80, 80, 80));

        found = foundRect.area() > minimumArea;

        if (found) {
            drawRect(foundRect, new Scalar(0, 255, 0));
            Imgproc.putText(displayMat, "Rings", foundRect.tl(),0,1,new Scalar(255,255,255));
        }
        return displayMat;
    }

}
