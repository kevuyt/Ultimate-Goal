package Library4997.MasqMotors;

/**
 * Created by Archish on 5/5/18.
 */

public enum MasqMotorModel {
    ORBITAL20, NEVEREST40, NEVEREST60, USDIGITAL_E4T, REVHDHEX40, NEVERREST_CLASSIC, NEVERREST256, REVHDHEX20;
        public static double DEFAULT_CPR = 537.6;
        public static double CPR(MasqMotorModel motorModel) {
            switch (motorModel){
                case ORBITAL20:
                    return 537.6;
                case NEVEREST40:
                    return 1120;
                case NEVEREST60:
                    return 1680;
                case USDIGITAL_E4T:
                    return 1440;
                case NEVERREST_CLASSIC:
                    return 6600;
                case NEVERREST256:
                    return 4400;
                case REVHDHEX40:
                    return 2240;
                case REVHDHEX20:
                    return 560;
            }
            return DEFAULT_CPR;
        }
        public double CPR () {
            return CPR(this);
        }
        public static int DEFAULT_RPM = 150;
        public static int RPM(MasqMotorModel motorModel) {
            switch (motorModel) {
                case ORBITAL20:
                    return 340;
                case NEVEREST40:
                    return 160;
                case NEVEREST60:
                    return 105;
                case REVHDHEX40:
                    return 150;
                case NEVERREST_CLASSIC:
                    return 28;
                case REVHDHEX20:
                    return 300;
            }
            return DEFAULT_RPM;
        }
        public int RPM () {
            return RPM(this);
        }
}