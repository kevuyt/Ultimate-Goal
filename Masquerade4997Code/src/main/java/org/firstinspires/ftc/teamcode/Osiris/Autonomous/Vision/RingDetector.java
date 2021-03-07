package org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import Library4997.MasqVision.MasqCVDetector;
import Library4997.MasqVision.filters.GrayscaleFilter;
import Library4997.MasqVision.filters.LumaFilter;
import Library4997.MasqVision.filters.MasqCVColorFilter;

/**
 * Created by Keval Kataria on 3/6/2021
 */
public class RingDetector extends MasqCVDetector {
    private MasqCVColorFilter lumaFilter = new LumaFilter(150);

    @Override
    public Mat processFrame(Mat input) {
        cropMat(input, tl, br);

        workingMat = input.clone();
        displayMat = input.clone();

        List<MatOfPoint> contoursBright = findContours(lumaFilter, workingMat.clone());
        List<Rect> rectsBright = contoursToRects(contoursBright);
        List<List<Rect>> listsOfBrightBlobs = groupIntoBlobs(rectsBright,10);
        Rect[] rings = chooseTwoRects(listsOfBrightBlobs);
        Rect bestRect = rings[0];
        Rect second = rings[0];

        found = bestRect.area() > minimumArea;
        found2 = second.area() > minimumArea;

        if (found) {
            drawRect(bestRect, new Scalar(0, 255, 0), false);
            drawCenterPoint(getCenterPoint(bestRect), new Scalar(0, 255, 0));
            foundRect = bestRect;
        }
        if (found2) {
            drawRect(second, new Scalar(0, 255, 0), false);
            drawCenterPoint(getCenterPoint(second), new Scalar(0, 255, 0));
            secondRect = second;
        }
        return displayMat;
    }
}