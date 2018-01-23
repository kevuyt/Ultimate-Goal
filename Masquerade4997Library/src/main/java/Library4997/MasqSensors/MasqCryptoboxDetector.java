package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqOpenCV.CameraViewDisplay;
import Library4997.MasqOpenCV.detectors.CryptoboxDetector;

/**
 * Created by Archish on 1/23/18.
 */

public class MasqCryptoboxDetector {
    private CryptoboxDetector cryptoboxDetector;
    public MasqCryptoboxDetector (HardwareMap hardwareMap) {
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        cryptoboxDetector.rotateMat = false;
        cryptoboxDetector.enable();
    }
    public boolean cryptoboxDetected () {return cryptoboxDetector.isCryptoBoxDetected();}
    public boolean columnDetected() {return cryptoboxDetector.isColumnDetected();}
    public int leftColumnPosition() {return cryptoboxDetector.getCryptoBoxLeftPosition();}
    public int centerColumnPosition() {return cryptoboxDetector.getCryptoBoxCenterPosition();}
    public int rightColumnPosition() {return cryptoboxDetector.getCryptoBoxRightPosition();}
}
