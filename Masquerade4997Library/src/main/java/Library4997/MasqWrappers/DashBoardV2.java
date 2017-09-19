package Library4997.MasqWrappers;

/**
 * Created by Archish on 9/19/17.
 */


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;

import Library4997.MasqHardware;

/**
 * This is a telemetry wrapper class.
 * It provides additional functionality such as stickied messages.
 * It supports multiple types of inputs, including MasqHardware objects.
 */

public class DashBoardV2 {

    private final Telemetry telemetry;

    private LinkedHashMap<String, Object> sticky, line;

    public DashBoardV2(Telemetry t) {
        telemetry = t;
        telemetry.clearAll();

        sticky = new LinkedHashMap<>();
        line = new LinkedHashMap<>();
    }

    public void addSticky(String key, Object message) {sticky.put(key, message);}
    public void addLine(String key, Object message) {line.put(key, message);}

    public void addSticky(final MasqHardware hardware) {
        int dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i ++) {
            sticky.put(hardware.getName(), hardware.getDash()[i]);
        }
    }

    public void addLine(final MasqHardware hardware) {
        int dashLength = hardware.getDash().length;
        for (int i = 0; i < dashLength; i ++) {
            sticky.put(hardware.getName(), hardware.getDash()[i]);
        }
    }

    public void update() {
        for (String key : sticky.keySet()) {
            Object message = sticky.get(key);
                telemetry.addData(key, message);
        }

        for (String key : line.keySet()) {
            Object message = line.get(key);
                telemetry.addData(key, message);
        }

        telemetry.update();
    }

}