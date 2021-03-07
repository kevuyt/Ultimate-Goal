package org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision;

import Library4997.MasqVision.MasqCVDetector;

import static java.lang.Math.abs;

/**
 * Created by Keval Kataria on 11/25/2020
 */
public class ZoneFinder {
    public enum TargetZone {A,B,C}

    public static TargetZone findZone (MasqCVDetector detector) {
        HeightDetector heightDetector = (HeightDetector) detector;
        if (abs(heightDetector.getTop()- heightDetector.getBottom()) > 15) return TargetZone.B;
        else if (abs(((heightDetector.getTop() + heightDetector.getBottom()) / 2 - heightDetector.getControl())) > 10) return TargetZone.C;
        else return TargetZone.A;
    }
}