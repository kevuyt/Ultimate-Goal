package DogeCV.detectors.skystone;

import org.opencv.core.Mat;

import DogeCV.detectors.DogeCVDetector;


public class SkystoneDetector extends DogeCVDetector {
    private Mat input = null;
    public SkystoneDetector() {
        detectorName = "Skystone Detector";
    }
    public void setInput(Mat input) {
        this.input = input;
    }
    @Override
    public Mat process(Mat input) {
        if (this.input != null) input = this.input;


        return input;
    }

    @Override
    public void useDefaults() {

    }
}