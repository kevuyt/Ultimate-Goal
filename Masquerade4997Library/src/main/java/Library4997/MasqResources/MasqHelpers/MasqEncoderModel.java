package Library4997.MasqResources.MasqHelpers;

/**
 * Created by Archish on 5/5/18.
 */

public enum  MasqEncoderModel {
        NEVEREST20, NEVEREST40, NEVEREST60, USDIGITAL_E4T;
        public static double DEFAULT_CPR = 0;
        public static double CPR(MasqEncoderModel motorModel) {
            switch (motorModel){
                case NEVEREST20:
                    return 537.6;
                case NEVEREST40:
                    return 1120;
                case NEVEREST60:
                    return 1680;
                case USDIGITAL_E4T:
                    return 1440;
            }
            return DEFAULT_CPR;
        }
        public double CPR () {
            return CPR(this);
        }
        public static int DEFAULT_RPM = 0;
        public static int RPM(MasqEncoderModel motorModel) {
            switch (motorModel) {
                case NEVEREST20:
                    return 340;
                case NEVEREST40:
                    return 160;
                case NEVEREST60:
                    return 105;
            }
            return DEFAULT_RPM;
        }
        public int RPM () {
            return RPM(this);
        }
}
