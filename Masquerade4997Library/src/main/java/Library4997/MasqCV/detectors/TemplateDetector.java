package Library4997.MasqCV.detectors;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Keval Kataria on 6/1/2020
 */
public class TemplateDetector extends MasqCVDetector {
    /**
     create filters
     */
    //private MasqCVColorFilter blackFilter = new GrayscaleFilter(0, 50);

    @Override
    public Mat process(Mat input) {
        cropMat(input,tl, br);

        workingMat = input.clone();
        displayMat = input.clone();

        /**
         find all contours for each filter and select the best rectange
         */

        /*List<MatOfPoint> contoursBlack = findContours(blackFilter, workingMat.clone());
        List<Rect> rectsBlack = contoursToRects(contoursBlack);
        List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlack);
        foundRect = chooseBestBlack(listOfBlackBlobs);*/

        /**
        draw the contours on the displayMat (optional)
         */
        //drawContours(contoursBlack, new Scalar(80, 80, 80));

        found = foundRect.area() > minimumArea;

        if (found) {
            drawRect(foundRect, new Scalar(0, 255, 0));
            drawCenterPoint(getCenterPoint(foundRect), new Scalar(0, 255, 0));
            Imgproc.putText(displayMat, "Chosen", foundRect.tl(),0,1,new Scalar(255,255,255));
        }
        return displayMat;
    }
}
