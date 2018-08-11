package Library4997.MasqResources.MasqHelpers;

/**
 * Created by Archish on 5/5/18.
 */

public enum  MasqEncoderModel {
        NEVEREST20, NEVEREST40, NEVEREST60, USDIGITAL_E4T;
        public double CPR;
        public double DEFAULT_CPR = 0;
        public double CPR(MasqEncoderModel motorModel) {
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
        public int RPM;
        public int DEFAULT_RPM = 0;
        public int RPM(MasqEncoderModel motorModel) {
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
}
