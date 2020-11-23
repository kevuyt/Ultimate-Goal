package org.firstinspires.ftc.teamcode.Testing;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import Library4997.MasqCV.detectors.MasqCVDetector;
import Library4997.MasqCV.filters.CbColorFilter;
import Library4997.MasqCV.filters.HSVRangeFilter;
import Library4997.MasqCV.filters.MasqCVColorFilter;

/**
 * Created by Keval Kataria on 6/1/2020
 */
public class TestDetector extends MasqCVDetector {
    /*private MasqCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
    private MasqCVColorFilter redFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
    private MasqCVColorFilter blueFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE);
    private MasqCVColorFilter whiteFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.WHITE);*/
    private MasqCVColorFilter orangeFilter = new CbColorFilter(60,80);

    @Override
    public Mat processFrame(Mat input) {
        workingMat = input.clone();
        displayMat = input.clone();

        /*List<MatOfPoint> contoursYellow = findContours(yellowFilter, workingMat.clone());
        List<Rect> rectsYellow = contoursToRects(contoursYellow);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rectsYellow,10);
        foundRect = chooseBestRect(listOfBlobs);*/

        /*List<MatOfPoint> contoursRed = findContours(redFilter, workingMat.clone());
        List<Rect> rectsRed = contoursToRects(contoursRed);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rectsRed,50);
        foundRect = chooseBestRect(listOfBlobs);*/

        /*List<MatOfPoint> contoursBlue = findContours(blueFilter, workingMat.clone());
        List<Rect> rectsBlue = contoursToRects(contoursBlue);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rectsBlue,50);
        foundRect = chooseBestRect(listOfBlobs);*/

        /*List<MatOfPoint> contoursWhite = findContours(whiteFilter, workingMat.clone());
        List<Rect> rectsWhite = contoursToRects(contoursWhite);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rectsWhite,50);
        foundRect = chooseBestRect(listOfBlobs);*/

        List<MatOfPoint> contours = findContours(orangeFilter, workingMat.clone());
        List<Rect> totalRects = contoursToRects(contours);
        List<Rect> rectsInBound = filterByBound(totalRects, new Rect(tl,br));
        drawRect(new Rect(tl,br), new Scalar(0,0,240),false);
        List<List<Rect>> listOfBlobs = groupIntoBlobs(rectsInBound,5);
        foundRect = chooseBestRect(listOfBlobs);
        //drawContours(contoursYellow, new Scalar(220,220,0));
        //drawContours(contoursRed, new Scalar(220,0,0));
        //drawContours(contoursBlue, new Scalar(0,0,220));
        //drawContours(contoursWhite, new Scalar(220,220,220));
        drawContours(contours, new Scalar(150,0,0));

        found = foundRect.area() > minimumArea;

        if (found) {
            drawRect(foundRect, new Scalar(0,0,255), false);
            drawCenterPoint(getCenterPoint(foundRect), new Scalar(0, 255, 0));
            Imgproc.putText(displayMat, "Chosen", foundRect.tl(),0,1,new Scalar(255,255,255));
        }
        return displayMat;
    }
}