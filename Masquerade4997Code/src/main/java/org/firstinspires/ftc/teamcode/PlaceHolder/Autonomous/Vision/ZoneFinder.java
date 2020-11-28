package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision;

import Library4997.MasqCV.detectors.MasqCVDetector;

import static java.lang.Math.abs;

/**
 * Created by Keval Kataria on 11/25/2020
 */
public class ZoneFinder {
    public enum TargetZone {
        A,B,C
    }
    public static TargetZone findZone (MasqCVDetector detector) {
        RingDetector ringDetector = (RingDetector) detector;
        if (abs(ringDetector.getAverage() - ringDetector.getControl()) < 1.75) return TargetZone.B;
        else if (ringDetector.getAverage() > ringDetector.getControl()) return TargetZone.C;
        else return TargetZone.A;
    }
}
