package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqOpenCV.CameraViewDisplay;
import Library4997.MasqOpenCV.detectors.CryptoboxDetector;

/**
 * Created by Archish on 1/23/18.
 */

public class MasqCryptoboxDetector implements MasqHardware {
    private CryptoboxDetector cryptoboxDetector;
    public MasqCryptoboxDetector (HardwareMap hardwareMap, MasqOpenCV.Alliance alliance) {
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        setAlliance(alliance);
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        cryptoboxDetector.rotateMat = false;
        cryptoboxDetector.enable();
    }
    public void setAlliance(MasqOpenCV.Alliance alliance) {
        if (alliance == MasqOpenCV.Alliance.BLUE) cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE;
        else cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
    }
    public boolean cryptoboxDetected () {return cryptoboxDetector.isCryptoBoxDetected();}
    public boolean columnDetected() {return cryptoboxDetector.isColumnDetected();}
    public int leftColumnPosition() {return cryptoboxDetector.getCryptoBoxLeftPosition();}
    public int centerColumnPosition() {return cryptoboxDetector.getCryptoBoxCenterPosition();}
    public int rightColumnPosition() {return cryptoboxDetector.getCryptoBoxRightPosition();}

    @Override
    public String getName() {
        return "CRYPTOBOX DETECTOR";
    }

    @Override
    public String[] getDash() {
        return new String[] {
                "CRYPOTBOX DETECTED: "  + Boolean.toString(cryptoboxDetected()),
                "IS COLUMN DETECTED: " + Boolean.toString(columnDetected()),
                "COLUMN: LEFT: POSITION " + leftColumnPosition(),
                "COLUMN: CENTER: POSITION " + centerColumnPosition(),
                "COLUMN: RIGHT: POSITION " + rightColumnPosition()
        };
    }
}
