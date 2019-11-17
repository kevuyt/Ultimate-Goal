package DogeCV.detectors.skystone;

import org.opencv.core.Mat;

import DogeCV.detectors.DogeCVDetector;


public class SkystoneDetector extends DogeCVDetector {
<<<<<<< HEAD
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
    private int top, bottom, left, right;

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

=======
>>>>>>> origin/master

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