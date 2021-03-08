package org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision;

import Library4997.MasqMath.MasqPoint;
import Library4997.MasqVision.MasqCVDetector;

import static Library4997.MasqUtils.getCenterPoint;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by Keval Kataria on 11/25/2020
 */
public class ZoneFinder {
    private static double ratio = 1;
    private static double x0 = 0;
    private static double y0 = 0;
    private static MasqPoint ring1 = new MasqPoint(0,0), ring2 = new MasqPoint(0,0);
    private static RingDetector ringDetector;

    public enum TargetZone {A,B,C}

    public static TargetZone findZone (MasqCVDetector detector) {
        ringDetector = (RingDetector) detector;
        if (abs(ringDetector.getTop()- ringDetector.getBottom()) > 15) return TargetZone.B;
        else if (abs(((ringDetector.getTop() + ringDetector.getBottom()) / 2 - ringDetector.getControl())) > 10) return TargetZone.C;
        else return TargetZone.A;
    }

    public static MasqPoint[] findRings(MasqCVDetector detector) {
        ringDetector = (RingDetector) detector;
        if(ringDetector.isFound()) {
            ring1 = new MasqPoint(getCenterPoint(ringDetector.getFoundRect()).x - 480 + x0,
                    sqrt(pow(ringDetector.getFoundRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getFoundRect()).x, 2)) + y0);
            ring2 = new MasqPoint(getCenterPoint(ringDetector.getSecondRect()).x - 480 + x0,
                    sqrt(pow(ringDetector.getSecondRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getSecondRect()).x, 2)) + y0);
        }
        else if(ringDetector.isFound2()) ring1 = new MasqPoint(getCenterPoint(ringDetector.getSecondRect()).x - 480 + x0,
                sqrt(pow(ringDetector.getSecondRect().height * ratio, 2) - pow(getCenterPoint(ringDetector.getSecondRect()).x, 2)) + y0);
        return new MasqPoint[] {ring1, ring2};
    }

    public void setRatio(double ratio) {ZoneFinder.ratio = ratio;}
    public void setDistances(double x0, double y0) {
        ZoneFinder.x0 = x0;
        ZoneFinder.y0 = y0;
    }
}