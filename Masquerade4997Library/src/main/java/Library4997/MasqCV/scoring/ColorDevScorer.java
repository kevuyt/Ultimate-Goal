package Library4997.MasqCV.scoring;

import com.disnodeteam.dogecv.math.MathFTC;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;


/**
 * Created by LeviG on 10/7/2018.
 */

public class ColorDevScorer extends MasqCVScorer {

    private MatOfDouble std = new MatOfDouble();
    private MatOfDouble mean = new MatOfDouble();

    /**
     * @param input - Input mat
     * @return - Difference from perfect score
     */
    @Override
    public double calculateScore(Mat input) {
        Core.meanStdDev(input, mean, std);
        return MathFTC.mean(std.get(0,0));
    }
}
