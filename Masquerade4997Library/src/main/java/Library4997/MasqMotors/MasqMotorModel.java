package Library4997.MasqMotors;

/**
 * Created by Archish on 5/5/18.
 */

public enum MasqMotorModel {
    ORBITAL20, NEVEREST40, NEVEREST60, USDIGITAL_E4T, REVHDHEX40, NEVERREST_CLASSIC, NEVERREST256, REVHDHEX20, REVTHROUGHBORE, NEVERREST37;
        public static double DEFAULT_CPR = 537.6;
        public static double CPR(MasqMotorModel motorModel) {
            switch (motorModel){
                case ORBITAL20:
                case REVHDHEX20:
                    return 560;
                case NEVEREST40:
                case REVHDHEX40:
                    return 1120;
                case NEVEREST60:
                    return 1680;
                case USDIGITAL_E4T:
                    return 1440;
                case NEVERREST_CLASSIC:
                    return 28;
                case NEVERREST256:
                    return 4400;
                case REVTHROUGHBORE:
                    return 8192;
                case NEVERREST37:
                    return 44.4;
            }
            return DEFAULT_CPR;
        }
        public double CPR () {return CPR(this);}
        public static int DEFAULT_RPM = 150;
        public static int RPM(MasqMotorModel motorModel) {
            switch (motorModel) {
                case ORBITAL20:
                    return 315;
                case NEVEREST40:
                    return 160;
                case NEVEREST60:
                    return 105;
                case REVHDHEX40:
                    return 150;
                case NEVERREST_CLASSIC:
                    return 6600;
                case REVHDHEX20:
                    return 300;
                case NEVERREST37:
                    return 1780;
            }
            return DEFAULT_RPM;
        }
        public int RPM () {return RPM(this);}
}