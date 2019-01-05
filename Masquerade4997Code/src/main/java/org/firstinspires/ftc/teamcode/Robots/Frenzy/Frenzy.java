package org.firstinspires.ftc.teamcode.Robots.Frenzy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqPIDPackage;
import Library4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 1/5/19.
 * Project: MasqLib
 */
public class Frenzy extends MasqRobot {
    @Override
    public void mapHardware(HardwareMap hardwareMap) {

    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}
