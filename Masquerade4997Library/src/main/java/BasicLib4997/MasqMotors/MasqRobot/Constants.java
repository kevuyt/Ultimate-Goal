package BasicLib4997.MasqMotors.MasqRobot;
/**
 * These are the constants used in PID
 */
public interface Constants {
    double
            KP_TURN = 0.004,
            KI_TURN = 0.06,
            KD_TURN = 0.000008,
            ID = 1;
    double
            KP_STRAIGHT = 0.02,
            KI_STRAIGHT = 0,
            KD_STRAIGHT = 0;
    double shooterPowerAuto = -0.6;
    double tickForAndymark = 1120;
    double wheelDiameter = 4;
    double cmToInches = 2.54;
    double gearRatio = 1.5;
    double CLICKS_PER_CM = ((tickForAndymark / (wheelDiameter * cmToInches)) / Math.PI) / gearRatio;
}

