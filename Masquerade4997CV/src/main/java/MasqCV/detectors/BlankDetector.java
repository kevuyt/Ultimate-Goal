package MasqCV.detectors;

import org.opencv.core.Mat;

/**
 * Created by Victo on 12/17/2017.
 */

public class BlankDetector extends MasqCVDetector {
    @Override
    public Mat process(Mat input) {
        // Process frame
        return input;
    }

    @Override
    public void useDefaults() {
        // Add in your scorers here.
    }
}
