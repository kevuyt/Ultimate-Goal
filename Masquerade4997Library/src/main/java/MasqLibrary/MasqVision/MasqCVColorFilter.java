package MasqLibrary.MasqVision;

import org.opencv.core.Mat;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public abstract class MasqCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}