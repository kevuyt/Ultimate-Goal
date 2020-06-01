package Library4997.MasqCV.detectors;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqCV.MasqCVUtils;
import Library4997.MasqCV.scoring.MasqCVScorer;
import Library4997.MasqWrappers.DashBoard;



/**
 * Created by Keval Kataria on 9/10/2018.
 */

public abstract class MasqCVDetector extends OpenCvPipeline {

    public abstract Mat process(Mat input);

    protected int minimumArea = 100;
    protected int imageWidth = 320;
    protected int imageHeight = 240;

    public Rect foundRect = new Rect();

    private List<MasqCVScorer> scorers = new ArrayList<>();
    private Mat output = new Mat();
    protected Mat workingMat;
    protected Mat displayMat;
    public double maxDifference = 10;
    
    protected boolean found = false;

    public String detectorName = "MasqCV Detector";

    protected Point tl, br;
    public int offset = 0;


    public void setSpeed(MasqCVUtils.DetectionSpeed speed){
    }
    
    public boolean isDetected(){
        return found;
    }

    public void addScorer(MasqCVScorer newScorer){
        scorers.add(newScorer);
    }

    public double calculateScore(Mat input){
        double totalScore = 0;

        for(MasqCVScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }


    @Override
    public final Mat processFrame(Mat input) {

        Log.d("MasqCVDetector", "Input mat size:" + input.size());
        input.copyTo(output);

        if(output.empty()){
            return input;
        }

        output = process(output);

        return output;
    }

    @Override
    public void onViewportTapped() {}


    public Mat cropMat(Mat input, Point tl, Point br)  {
        if (!(tl == null || br == null || tl.x >= input.width() || tl.y >= input.height() || tl.x < 0 || tl.y < 0 || br.x > input.width() || br.y > input.height() || br.x <= 0 || br.y <= 0)) {
            Imgproc.rectangle(input,new Point(tl.x, tl.y),new Point(br.x, br.y),new Scalar(0), -1);
        }
        else {
            DashBoard.getDash().create("Cropping failed due to invalid cropping margins");
        }
        return input;
    }
    private Rect chooseBestRect(List<List<Rect>> listOfBlobs) {
        Rect bestRect = new Rect();
        for (List<Rect> blob : listOfBlobs) {
            Rect blobBound = boundingRect(blob);
            drawRect(blobBound, new Scalar(0, 150, 0));

            if (blobBound.area() > bestRect.area()) {
                bestRect = blobBound;
            }
        }
        return bestRect;
    }
    private Rect boundingRect(List<Rect> rects) {
        int minX = 999;
        int minY = 999;
        int maxX = 0;
        int maxY = 0;

        for (Rect rect : rects) {
            minX = Math.min(rect.x, minX);
            minY = Math.min(rect.y, minY);
            maxX = Math.max(rect.x + rect.width, maxX);
            maxY = Math.max(rect.y + rect.height, maxY);
        }

        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }
        private void drawRect(Rect rect, Scalar color) {
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), color, 2);
        }

        private void drawCircle(Point point, Scalar color) {
            Imgproc.circle(displayMat, point, 2, color);
        }

    private List<Rect> filterByBound(List<Rect> rects, Rect boundingRect) {
        List<Rect> rectsInsideBound = new ArrayList<>();
        for (Rect rect : rects) {
            if (boundingRect.contains(getCenterPoint(rect))) {
                rectsInsideBound.add(rect);
            }
        }
        return rectsInsideBound;
    }
    public Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
    }

    public Rect getFoundRect() {return foundRect;}

    public void setClippingMargins(Point tl, Point br) {
        this.tl = tl;
        this.br = br;
        imageWidth = (int) (br.x - tl.x);
        imageHeight = (int) (tl.y - br.y);
        offset = (int) tl.x;
    }
    public void setClippingMargins(int top, int left, int bottom, int right) {
        tl = new Point(left, top);
        br = new Point(320 - right,240 - bottom);
        imageWidth = 320 - right - left;
        imageHeight = top - 240 + bottom;
        offset = left;
    }
}
