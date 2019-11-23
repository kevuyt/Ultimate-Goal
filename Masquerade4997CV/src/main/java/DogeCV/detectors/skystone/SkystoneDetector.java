package DogeCV.detectors.skystone;

import org.opencv.core.Mat;
import org.opencv.core.Rect;

import DogeCV.detectors.DogeCVDetector;
import Library4997.MasqWrappers.DashBoard;


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
        if (this.input != null) {
            input = this.input;
            int width = input.width();
            Mat mat1 = input.submat(new Rect(0, 0, width/3, input.height()));
    /*Mat mat2 = input.submat(new Rect(width/3, 0, width/3, input.height()));
    Mat mat3 = input.submat(new Rect( 2 * (width/3), 0, width/3, input.height()));
    List<Mat> src = Arrays.asList(mat1, mat2, mat3);
    Core.hconcat(src, input);*/
            input = mat1;
            DashBoard.getDash().create("Hello");
        }
        return input;
    }

    @Override
    public void useDefaults() {

    }
}