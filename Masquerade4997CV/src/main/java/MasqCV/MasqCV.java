package MasqCV;

/**
 * Contains global values and types
 */
public class MasqCV {
    public enum DetectionSpeed {
        VERY_FAST,
        FAST,
        BALANCED,
        SLOW,
        VERY_SLOW
    }

    public enum AreaScoringMethod {
        MAX_AREA,
        PERFECT_AREA,
        COLOR_DEVIATION
    }

    public enum CameraMode {
        BACK,
        FRONT,
        WEBCAM
    }

    public enum VuMark {
        NONE,
        BLUE_ROVER,
        RED_FOOTPRINT,
        FRONT_CRATERS,
        BACK_SPACE
    }
}
