package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Archish on 1/23/18.
 */

public class MasqOpenCV {
    MasqCryptoboxDetector cryptoboxDetector;
    public MasqOpenCV (HardwareMap hardwareMap) {
        cryptoboxDetector = new MasqCryptoboxDetector(hardwareMap);
    }

    public MasqCryptoboxDetector getCryptoboxDetector() {
        return cryptoboxDetector;
    }
}
