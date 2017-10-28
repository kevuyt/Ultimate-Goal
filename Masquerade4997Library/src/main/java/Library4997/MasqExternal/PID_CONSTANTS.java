package Library4997.MasqExternal;
/**
 * These are the constants used in PID
 */
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
    double MAX_RATE = 3100;
    double TICKS_PER_ROTATION = 1120;
    double wheelDiameter = 4;
    double cmToInches = 2.54;
    double gearRatio = .5;
    double CLICKS_PER_INCH = (Math.PI * wheelDiameter)/(TICKS_PER_ROTATION * gearRatio);
}

