package Library4997.MasqSensors;

import com.disnodeteam.masqcv.CameraViewDisplay;
import com.disnodeteam.masqcv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqHardware;


/**
 * Created by Archish on 1/23/18.
 */

public class MasqJewelDetector implements MasqHardware{
    JewelDetector jewelDetector;
    public MasqJewelDetector (HardwareMap hardwareMap) {
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA;
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.enable();
    }
    public String getCurrentOrder () {
        return jewelDetector.getCurrentOrder().toString();
    }

    @Override
    public String getName() {
        return "JEWEL DETECTOR";
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "JEWEL ORDER: " + getCurrentOrder()
        };
    }
}