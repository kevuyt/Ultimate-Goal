package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems;

import Library4997.MasqResources.MasqUtils;
import MasqCV.detectors.MasqCVDetector;
import MasqCV.detectors.skystone.SkystoneDetector;

/**
 * Created by Archishmaan Peyyety on 2019-12-17.
 * Project: MasqLib
 */
public class CVInterpreter {
    public enum SkystonePosition {
        LEFT("LEFT"), MIDDLE("MIDDLE"), RIGHT("RIGHT");
        public final String position;
        SkystonePosition (String value) {this.position = value;}
    }
    public static SkystonePosition getPosition(MasqCVDetector genDetector) {
        SkystoneDetector detector = (SkystoneDetector) genDetector;
        if ((MasqUtils.getCenterPoint(detector.foundRectangle()).x - detector.offset) < detector.getImageWidth()/3) return SkystonePosition.LEFT;
        else if ((MasqUtils.getCenterPoint(detector.foundRectangle()).x - detector.offset) < (2 * detector.getImageWidth()/3)) return SkystonePosition.MIDDLE;
        else return SkystonePosition.RIGHT;
    }
}
