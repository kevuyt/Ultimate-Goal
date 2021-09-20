package org.firstinspires.ftc.teamcode.Osiris.Autonomous;

import org.opencv.core.*;

import MasqLibrary.MasqVision.*;

import static java.lang.System.nanoTime;
import static org.firstinspires.ftc.teamcode.Osiris.Autonomous.MarkerDetector.MarkerLocation.*;
import static org.opencv.core.Core.*;

/**
 * Created by Keval Kataria on 9/18/2021
 */

public class MarkerDetector extends MasqCVDetector {
    double left, mid, right;
    double prevTime = 0;
    MarkerLocation location;

    @Override
    public Mat process(Mat input) {
        double time = nanoTime();

        if(time - prevTime > 1e6) {
            prevTime = time;
            workingMat = input.clone();
            displayMat = input.clone();

            extractChannel(workingMat, workingMat, 1);

            Rect leftRect = new Rect(tl, new Point((br.x- tl.x) / 3 + tl.x, br.y));
            Rect midRect = new Rect(new Point((br.x- tl.x) / 3 + tl.x, tl.y),
                    new Point((2 * (br.x- tl.x) / 3) + tl.x, br.y));
            Rect rightRect = new Rect(new Point(2 * (br.x- tl.x) / 3 + tl.x, tl.y), br);

            left = mean(workingMat.clone().submat(leftRect)).val[0];
            mid = mean(workingMat.clone().submat(midRect)).val[0];
            right = mean(workingMat.clone().submat(rightRect)).val[0];

            workingMat.release();

            if (left > right) {
                if (left > mid) {
                    foundRect = leftRect;
                    location = LEFT;
                }
                else {
                    foundRect = midRect;
                    location = MID;
                }
            }
            else if (right > mid) {
                foundRect = rightRect;
                location = RIGHT;
            }
            else {
                foundRect = midRect;
                location = MID;
            }

            drawRect(foundRect, new Scalar(255, 0, 0), false);
        }

        return displayMat;
    }

    public enum MarkerLocation {LEFT, MID, RIGHT}
    public MarkerLocation getLocation() {return location;}
}