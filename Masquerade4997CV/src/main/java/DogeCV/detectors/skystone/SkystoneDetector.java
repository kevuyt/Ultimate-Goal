package DogeCV.detectors.skystone;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import DogeCV.DogeCV;
import DogeCV.detectors.DogeCVDetector;
import DogeCV.filters.DogeCVColorFilter;
import DogeCV.filters.GrayscaleFilter;
import DogeCV.filters.LeviColorFilter;
import DogeCV.scoring.MaxAreaScorer;
import DogeCV.scoring.PerfectAreaScorer;
import DogeCV.scoring.RatioScorer;
import Library4997.MasqWrappers.DashBoard;


public class SkystoneDetector extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorer = new RatioScorer(1.25, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value


    // Results of the detector
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect
    int top, bottom, left, right;

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    public Point getScreenPosition() {
        return screenPosition;
    }

    public Rect foundRectangle() {
        return foundRect;
    }


    public SkystoneDetector() {
        detectorName = "Skystone Detector";
    }

    @Override
    public Mat process(Mat input) {
        /*input = cropMat(input, top, bottom, left, right);
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(255,30,30),2);


        // Current result
        Rect bestRect = foundRect;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }

        Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255,255,255), 1, Imgproc.LINE_4, 0);
        blackFilter.process(workingMat.clone(), blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(40,40,40),2);

        for(MatOfPoint cont : contoursBlack){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }
        if(bestRect != null) {
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        }
        else {
            found = false;
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }*/
        return input;
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }

    public void setCropSettings(int top, int bottom, int left, int right) {
        this.top = top;
        this.bottom = bottom;
        this.left = left;
        this.right = right;
    }

    private Mat cropMat(Mat input, int top, int bottom, int left, int right) {
        int x = (input.width() + left - right)/2, y = (input.height() + top - bottom)/2, width = input.width() - right - left, height = input.height() - top - bottom;
        //Rect rect = new Rect(input.width()/2, input.height()/2, input.width(), input.height());
        Rect rect = new Rect(x,y,width,height);
        DashBoard.getDash().clear();
        DashBoard.getDash().create(x);
        DashBoard.getDash().create(y);
        DashBoard.getDash().create(width);
        DashBoard.getDash().create(height);
        DashBoard.getDash().update();

        return input.submat(rect);
    }
}