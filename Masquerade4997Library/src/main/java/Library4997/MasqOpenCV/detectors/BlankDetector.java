package Library4997.MasqOpenCV.detectors;


import org.opencv.core.Mat;

import Library4997.MasqOpenCV.OpenCVPipeline;

/**
 * Created by Victo on 12/17/2017.
 */

public class BlankDetector extends OpenCVPipeline {
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        return rgba;
    }
}
