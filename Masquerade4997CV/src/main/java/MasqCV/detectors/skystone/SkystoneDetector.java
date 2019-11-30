package MasqCV.detectors.skystone;

import android.graphics.Point;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library4997.MasqWrappers.DashBoard;
import MasqCV.detectors.MasqCVDetector;
import MasqCV.filters.GrayscaleFilter;


public class SkystoneDetector extends MasqCVDetector {
    private GrayscaleFilter filter = new GrayscaleFilter(0,25);
    private Mat blackMask = new Mat();
    private Mat output = new Mat();
    private Mat input = null;
    private Mat hierarchy = new Mat();
    Rect stoneRect;
    Point stonePosition = new Point(0,0);
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
            input.copyTo(blackMask);
            input.copyTo(output);
            filter.process(input.clone(), blackMask);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(blackMask,contours,hierarchy, Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(output,contours,-1,new Scalar(255,30,30),2);
            stoneRect = new Rect();
            double bestDifference = Double.MAX_VALUE;
            for(MatOfPoint cont : contours){
                double score = calculateScore(cont);
                Rect rect = Imgproc.boundingRect(cont);
                Imgproc.rectangle(output, rect.tl(), rect.br(), new Scalar(0,0,255),2);
                if(score < bestDifference){
                    bestDifference = score;
                    stoneRect = rect;
                }
            }
                // Show chosen result
                Imgproc.rectangle(output, stoneRect.tl(), stoneRect.br(), new Scalar(255,0,0),4);
                Imgproc.putText(output, "Chosen", stoneRect.tl(),0,1,new Scalar(255,255,255));

                stonePosition = new Point(stoneRect.x, stoneRect.y);
/*
            int width = input.width();
            Mat mat1 = input.submat(new Rect(0, 0, width/3, input.height()));
            Mat mat2 = input.submat(new Rect(width/3, 0, width/3, input.height()));
            Mat mat3 = input.submat(new Rect( 2 * (width/3), 0, width/3, input.height()));
            List<Mat> src = Arrays.asList(mat1, mat2, mat3);
            Core.hconcat(src, input);
            input = mat1;
            DashBoard.getDash().create("Hello");*/
        }
        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            default: {
                return output;
            }
        }
    }

    @Override
    public void useDefaults() {

    }
    public SkystonePosition getSkystonePosition() {
        double x = stonePosition.x;
        if(x < 160) return SkystonePosition.LEFT;
        else if(x <= 320) return SkystonePosition.MIDDLE;
        else return SkystonePosition.RIGHT;
    }
    public enum SkystonePosition {
        LEFT,MIDDLE,RIGHT
    }
}