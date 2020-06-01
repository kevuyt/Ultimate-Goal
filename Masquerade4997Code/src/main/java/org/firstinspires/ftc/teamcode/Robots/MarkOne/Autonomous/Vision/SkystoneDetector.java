package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import Library4997.MasqCV.detectors.MasqCVDetector;
import Library4997.MasqCV.filters.GrayscaleFilter;
import Library4997.MasqCV.filters.MasqCVColorFilter;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.DashBoard;

public class SkystoneDetector extends MasqCVDetector {
    private MasqCVColorFilter blackFilter = new GrayscaleFilter(0, 50);




    @Override
    public Mat process(Mat input) {
        cropMat(input,tl, br);

        workingMat = input.clone();
        displayMat = input.clone();

        List<MatOfPoint> contoursBlack = findContours(blackFilter, workingMat.clone());
        List<Rect> rectsBlack = contoursToRects(contoursBlack);
        List<List<Rect>> listOfBlackBlobs = groupIntoBlobs(rectsBlack);
        Rect bestSkystoneRect = chooseBestBlack(listOfBlackBlobs);

        draw(contoursBlack, new Scalar(80, 80, 80));

        found = foundRect.area() > minimumArea;

        if (found) {
            draw(bestSkystoneRect, new Scalar(0, 255, 0));
            draw(getCenterPoint(bestSkystoneRect), new Scalar(0, 255, 0));
            foundRect = bestSkystoneRect;
            Imgproc.putText(displayMat, "Chosen", foundRect.tl(),0,1,new Scalar(255,255,255));
        }
                return displayMat;

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

    private List<MatOfPoint> findContours(MasqCVColorFilter filter, Mat mask) {
        filter.process(workingMat, mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }


    private void draw(Rect rect, Scalar color) {
        Imgproc.rectangle(displayMat, rect.tl(), rect.br(), color, 2);

    }

    private void draw(Point point, Scalar color) {
        Imgproc.circle(displayMat, point, 2, color);
    }

    private void draw(List<MatOfPoint> contours, Scalar color) {
        // Not draw contours for now, since can look messy
        Imgproc.drawContours(displayMat, contours, -1, color, 2);
    }

    private Rect chooseBestBlack(List<List<Rect>> listOfBlackBlobs) {
        Rect bestBlackRect = new Rect();
        for (List<Rect> blob : listOfBlackBlobs) {
            Rect blobBound = boundingRect(blob);
            draw(blobBound, new Scalar(0, 150, 0));

            if (blobBound.area() > bestBlackRect.area()) {
                bestBlackRect = blobBound;
            }
        }
        return bestBlackRect;
    }

    private  List<Rect> contoursToRects(List<MatOfPoint> contours) {
        List<Rect> rects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            rects.add(Imgproc.boundingRect(contour));
        }
        return rects;
    }

    private List<List<Rect>> groupIntoBlobs(List<Rect> rects) {
        List<List<Rect>> listOfBlobs = new ArrayList<>();
        List<Rect> unusedRects = new ArrayList<>(rects);

        while (!unusedRects.isEmpty()) {
            LinkedList<Rect> toProcess = new LinkedList<>();
            toProcess.add(unusedRects.remove(0));
            List<Rect> currentBlob = new ArrayList<>();
            while (!toProcess.isEmpty()) {
                Rect currentRect = toProcess.poll();
                currentBlob.add(currentRect);
            }
            listOfBlobs.add(currentBlob);
        }

        return listOfBlobs;
    }


    public Mat cropMat(Mat input, Point tl, Point br)  {
        if (!(tl == null || br == null || tl.x >= input.width() || tl.y >= input.height() || tl.x < 0 || tl.y < 0 || br.x > input.width() || br.y > input.height() || br.x <= 0 || br.y <= 0)) {
            Imgproc.rectangle(input,new Point(0,0),new Point(input.width(), tl.y), new Scalar(255,255,255), -1);
            Imgproc.rectangle(input, new Point(0,0), new Point(tl.x, input.height()), new Scalar(255,255,255), -1);
            Imgproc.rectangle(input, new Point(input.width(), input.height()), new Point(br.x, 0), new Scalar(255,255,255), -1);
            Imgproc.rectangle(input, new Point(input.width(), input.height()), new Point(0, br.y), new Scalar(255,255,255), -1);
        }
        else {
            DashBoard.getDash().create("Cropping failed due to invalid cropping margins");
        }
        return input;
    }

    public enum SkystonePosition {
        LEFT("LEFT"), MIDDLE("MIDDLE"), RIGHT("RIGHT");
        public final String position;
        SkystonePosition (String value) {this.position = value;}
    }
    public SkystonePosition getPosition() {
        if ((MasqUtils.getCenterPoint(foundRect).x - offset) < getImageWidth()/3) return SkystonePosition.LEFT;
        else if ((MasqUtils.getCenterPoint(foundRect).x - offset) < (2 * getImageWidth()/3)) return SkystonePosition.MIDDLE;
        else return SkystonePosition.RIGHT;
    }

    public int getImageWidth() {
        return imageWidth;
    }

    public int getImageHeight() {
        return imageHeight;
    }
}