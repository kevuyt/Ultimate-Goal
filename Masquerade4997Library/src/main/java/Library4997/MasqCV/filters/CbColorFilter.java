package Library4997.MasqCV.filters;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class CbColorFilter extends MasqCVColorFilter {
    Scalar lower;
    Scalar upper;

    public CbColorFilter(int lower, int upper) {
        this.lower = new Scalar(lower);
        this.upper = new Scalar(upper);
    }

    @Override
    public void process(Mat input, Mat mask) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input, input, 2);
        Core.inRange(input, lower, upper, mask);
        input.release();
    }
}
