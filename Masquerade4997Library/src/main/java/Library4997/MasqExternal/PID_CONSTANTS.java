package Library4997.MasqExternal;
/**
 * These are the constants used in PID
 */
@Deprecated
public interface PID_CONSTANTS {
    double
            KP_TURN = 0.005,
            KI_TURN = 0.0002,
            KD_TURN = 0,
            ID = 1;
    double
            KP_STRAIGHT = 0.03,
            KI_STRAIGHT = 0,
            KD_STRAIGHT = 0;
    double  KP_TELE = 0.1,
            KI_TELE = 0;
    double MAX_RATE = 160;
    double TICKS_PER_ROTATION = 537.6;
    double wheelDiameter = 4;
    double cmToInches = 2.54;
    double gearRatio = 2;
    double CLICKS_PER_INCH = ((TICKS_PER_ROTATION / (wheelDiameter * cmToInches)) / Math.PI) / gearRatio;
}

