package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Archish on 10/4/17.
 */

public class Hardware {
    public Servo left, right;
    HardwareMap hardwareMap;
    public void init (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        right = hardwareMap.servo.get("rightGlyph");
        left = hardwareMap.servo.get("letGlyph");
    }
}
