package org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision;

import org.opencv.core.Rect;

import Library4997.MasqMath.MasqPoint;
import Library4997.MasqMath.MasqWayPoint;
import Library4997.MasqVision.MasqCVDetector;

import static Library4997.MasqUtils.getCenterPoint;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by Keval Kataria on 3/6/2021
 */
public class RingFinder {
    private double ratio = 1;
    private double x0 = 0;
    private double y0 = 0;
    private MasqPoint ring1 = new MasqPoint(0,0), ring2 = new MasqPoint(0,0);

    public MasqPoint[] findRings (MasqCVDetector detector) {
        RingDetector ringDetector = (RingDetector) detector;
        if(ringDetector.isFound()) {
            ring1 = new MasqPoint(getCenterPoint(ringDetector.getFoundRect()).x - 640 + x0,
                    sqrt(pow(ringDetector.getFoundRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getFoundRect()).x, 2)) + y0);
            ring2 = new MasqPoint(getCenterPoint(ringDetector.getSecondRect()).x - 640 + x0,
                    sqrt(pow(ringDetector.getSecondRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getSecondRect()).x, 2)) + y0);
        }
        else if(ringDetector.isFound2()) ring1 = new MasqPoint(getCenterPoint(ringDetector.getSecondRect()).x - 640 + x0,
                sqrt(pow(ringDetector.getSecondRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getSecondRect()).x, 2)) + y0);
        return new MasqPoint[] {ring1, ring2};
    }

    public void setRatio(double ratio) {this.ratio = ratio;}
    public void setDistances(double x0, double y0) {
        this.x0 = x0;
        this.y0 = y0;
    }
}
