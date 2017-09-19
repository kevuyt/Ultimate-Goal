package Library4997.MasqWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import Library4997.MasqHardware;

/**
 * Created by Archish on 9/19/17.
 */

public class MasqController implements MasqHardware{
    Gamepad gamepad;
    String name;
    private boolean aPrev, bPrev, xPrev, yPrev,
                    leftBumperPrev, rightBumperPrev
                    = false;
    public MasqController(Gamepad g, String name){
        gamepad = g;
        this.name = name;
    }
    private double min = 0, max = 1;
    public boolean a() {return gamepad.a;}
    public boolean aOnRelease() {return a() && aPrev;}
    public boolean aOnPress() {return a() && !aPrev;}

    public boolean b() {return gamepad.b;}
    public boolean bOnRelease() {return b() && bPrev;}
    public boolean bOnPress() {return b() && !bPrev;}

    public boolean x() {return gamepad.x;}
    public boolean xOnRelease() {return !x() && xPrev;}
    public boolean xOnPress() {return x() && !xPrev;}

    public boolean y() {return gamepad.y;}
    public boolean yOnRelease() {return !y() && yPrev;}
    public boolean yOnPress() {return y() && !yPrev;}

    public boolean leftBumper() {return gamepad.left_bumper;}
    public boolean leftBumperOnRelease() {return !leftBumper() && leftBumperPrev;}
    public boolean leftBumperOnPress() {return leftBumper() && !leftBumperPrev;}

    public boolean rightBumper() {return gamepad.right_stick_button;}
    public boolean rightBumperOnRelease() {return !rightBumper() && rightBumperPrev;}
    public boolean rightBumperOnPress() {return rightBumper() && !rightBumperPrev;}

    public float left_stick_x(){return gamepad.left_stick_x;}
    public float left_stick_y() {return gamepad.left_stick_y;}
    public float right_stick_x() {return gamepad.right_stick_x;}
    public float right_stick_y() {return gamepad.right_stick_y;}

    public boolean dpad_up() {return gamepad.dpad_up;}
    public boolean dpad_down() {return gamepad.dpad_down;}
    public boolean dpad_left() {return gamepad.dpad_left;}
    public boolean dpad_right() {return gamepad.dpad_right;}

    public void setTriggerThersholds(double min, double max){this.min = min; this.max = max;}
    public void setTriggerThersholds (double min){ this.min = min; this.max = 1;}

    public boolean leftTriggerPressed() {return gamepad.left_trigger > min && gamepad.left_trigger < max;}
    public boolean rightTriggerPressed() {return gamepad.right_trigger >= min && gamepad.right_trigger <= max;}

    public float leftTrigger() {return gamepad.left_trigger;}
    public float rightTrigger() {return gamepad.right_trigger;}

    public void update () {
        aPrev = a(); bPrev = b();
        xPrev = x(); yPrev = y();
        leftBumperPrev = leftBumper(); rightBumperPrev = rightBumper();
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "A: " + Boolean.toString(a()),
                "aPress: " + Boolean.toString(aOnPress()),
                "aRelease: " + Boolean.toString(aOnRelease()),
                "B: " + Boolean.toString(b()),
                "bPress: " + Boolean.toString(bOnPress()),
                "bRelease: " + Boolean.toString(bOnRelease()),
                "X: " + Boolean.toString(x()),
                "xPress: " + Boolean.toString(xOnPress()),
                "xRelease: " + Boolean.toString(xOnRelease()),
                "Y: " + Boolean.toString(y()),
                "yPress: " + Boolean.toString(yOnPress()),
                "yRelease: " + Boolean.toString(yOnRelease()),
                "RB: " + Boolean.toString(rightBumper()),
                "rightBumperPressed: " + Boolean.toString(rightBumperOnPress()),
                "rightBumperRelease: " + Boolean.toString(rightBumperOnRelease()),
                "LB: " + Boolean.toString(leftBumper()),
                "leftBumperPressed: " + Boolean.toString(leftBumperOnPress()),
                "leftBumperRelease: " + Boolean.toString(leftBumperOnRelease()),
                "leftTrigger: " + Float.toString(leftTrigger()),
                "leftTriggerPress: " + Boolean.toString(leftTriggerPressed()),
                "rightTrigger: " + Float.toString(rightTrigger()),
                "rightTriggerPress: " + Boolean.toString(rightTriggerPressed()),
                "rightStickY: " + Double.toString(right_stick_y()),
                "rightStickX: " + Double.toString(right_stick_x()),
                "leftStickY: " + Double.toString(left_stick_y()),
                "leftStickX: " + Double.toString(left_stick_x()),
                "D_UP: " + Boolean.toString(dpad_up()),
                "D_DOWN: " + Boolean.toString(dpad_down()),
                "D_LEFT: " + Boolean.toString(dpad_left()),
                "D_RIGHT: " + Boolean.toString(dpad_right()),
        };
    }
}