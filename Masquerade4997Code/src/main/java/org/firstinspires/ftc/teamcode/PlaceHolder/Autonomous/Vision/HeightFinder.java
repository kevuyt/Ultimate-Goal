package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision;

import Library4997.MasqCV.detectors.MasqCVDetector;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class HeightFinder {
    public enum TargetZone {
        A, B, C
    }
    public static TargetZone findZone(MasqCVDetector genDetector) {
        RingDetector detector = (RingDetector) genDetector;
        if (!detector.isFound() || detector.getFoundRect().y < 10) return TargetZone.A;
        if (detector.getFoundRect().y < 40) return TargetZone.B;
        return TargetZone.C;
    }
}
