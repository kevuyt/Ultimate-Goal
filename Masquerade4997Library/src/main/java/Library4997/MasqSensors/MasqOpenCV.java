package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Archish on 1/23/18.
 */

public class MasqOpenCV {
    public enum Alliance {
        BLUE, RED
    }
    MasqCryptoboxDetector cryptoboxDetector;
    MasqJewelDetector jewelDetector;
    public MasqOpenCV (HardwareMap hardwareMap) {
        cryptoboxDetector = new MasqCryptoboxDetector(hardwareMap, Alliance.RED);
        jewelDetector = new MasqJewelDetector(hardwareMap);
    }
    public void setAlliance (Alliance alliance) {
        cryptoboxDetector.setAlliance(alliance);
    }
    public MasqCryptoboxDetector getCryptoboxDetector() {
        return cryptoboxDetector;
    }

    public MasqJewelDetector getJewelDetector() {
        return jewelDetector;
    }
}
