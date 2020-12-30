package Library4997.MasqVision.filters;

import org.opencv.core.Mat;

/**
 * Created by Victo on 1/1/2018.
 */

public abstract class MasqCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}
