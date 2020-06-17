package org.firstinspires.ftc.teamcode.MarkOne.Autonomous.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import Library4997.MasqCV.detectors.MasqCVDetector;
import Library4997.MasqCV.filters.GrayscaleFilter;
import Library4997.MasqCV.filters.MasqCVColorFilter;

public class SkystoneDetector extends MasqCVDetector {
    private MasqCVColorFilter blackFilter = new GrayscaleFilter(0, 50);

    @Override
    public Mat process(Mat input) {
        cropMat(input, tl, br);

        workingMat = input.clone();
        displayMat = input.clone();

        List<MatOfPoint> contoursBlack = findContours(blackFilter, workingMat.clone());
        List<Rect> rectsBlack = contoursToRects(contoursBlack);
        List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlack,10);
        Rect bestSkystoneRect = chooseBestRect(listOfBlackBlobs);

        drawContours(contoursBlack, new Scalar(80, 80, 80));

        found = bestSkystoneRect.area() > minimumArea;

        if (found) {
            drawRect(bestSkystoneRect, new Scalar(0, 255, 0));
            drawCenterPoint(getCenterPoint(bestSkystoneRect), new Scalar(0, 255, 0));
            foundRect = bestSkystoneRect;
            Imgproc.putText(displayMat, "Chosen", foundRect.tl(), 0, 1, new Scalar(255, 0,0));
        }
        return displayMat;

    }
}