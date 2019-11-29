package MasqCV.scoring;

import org.opencv.core.Mat;

/**
 * Created by Victo on 9/10/2018.
 */

public abstract class MasqCVScorer {
    public abstract double calculateScore(Mat input);
}
