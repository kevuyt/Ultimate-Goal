package Library4997;

import Library4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 3/3/17.
 */

public class MasqController implements MasqHardware{
    private MasqClock clock = new MasqClock();
    private String name;
    private double min, max;
    public MasqController(com.qualcomm.robotcore.hardware.Gamepad gamepad, String name){
        this.name = name;
        this.gamepad  = gamepad;
        instance = this;
    }
    public static MasqController getController(){
        return instance;
    }
    private static MasqController instance;
    private com.qualcomm.robotcore.hardware.Gamepad gamepad;
    public boolean a() {
        return gamepad.a;
    }
    public boolean x() {
        return gamepad.x;
    }
    public boolean y() {
        return gamepad.y;
    }
    public boolean b() {
        return gamepad.b;
    }
    public boolean aIsDoubleTapped () {
        boolean doubleTapped = false;
         if (apr() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = a();
        }
        return doubleTapped;
    }
    public boolean bIsDoubleTapped () {
        boolean doubleTapped = false;
        if (bpr() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = b();
        }
        return doubleTapped;
    }
    public boolean xIsDoubleTapped () {
        boolean doubleTapped = false;
        if (xpr() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = x();
        }
        return doubleTapped;
    }
    public boolean yIsDoubleTapped () {
        boolean doubleTapped = false;
        if (ypr() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = y();
        }
        return doubleTapped;
    }
    public boolean apr() {
        boolean pressed = false, released = false;
        while (a()) {
            released = false;
            pressed = true;
        }
        while (!a()) {
            released = true;
        }
        return pressed && released;
    }
    public boolean bpr() {
        boolean pressed = false, realeaed = false;
        while (b()) {
            realeaed = false;
            pressed = true;
        }
        while (!b()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }
    public boolean ypr() {
        boolean pressed = false, realeaed = false;
        while (b()) {
            realeaed = false;
            pressed = true;
        }
        while (!b()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }
    public boolean xpr() {
        boolean pressed = false, realeaed = false;
        while (x()) {
            realeaed = false;
            pressed = true;
        }
        while (!x()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }
    public void setTriggerThersholds(double min, double max){
        this.min = min;
        this.max = max;
    }
    public void setTriggerThersholds (double min){
        this.min = min;
        this.max = 1;
    }
    public float left_stick_x(){
        return gamepad.left_stick_x;
    }
    public float left_stick_y() {
     return gamepad.left_stick_y;
    }
    public float right_stick_x() {
        return gamepad.right_stick_x;
    }
    public float right_stick_y() {
        return gamepad.right_stick_y;
    }
    public boolean dpad_up() {
        return gamepad.dpad_up;
    }
    public boolean dpad_down() {
        return gamepad.dpad_down;
    }
    public boolean dpad_left() {
        return gamepad.dpad_left;
    }
    public boolean dpad_right() {
        return gamepad.dpad_right;
    }
    public boolean left_bumper() {
        return gamepad.left_bumper;
    }
    public boolean right_bumper() {
        return gamepad.right_bumper;
    }
    public boolean left_stick_button() {
        return gamepad.left_stick_button;
    }
    public boolean right_stick_button() {
        return gamepad.right_stick_button;
    }
    public boolean left_triggerb() {
        return gamepad.left_trigger > min && gamepad.left_trigger < max;
    }
    public boolean right_triggerb() {
        return gamepad.right_trigger >= min && gamepad.right_trigger <= max;

    }
    public float left_trigger() {
        return gamepad.left_trigger;
    }
    public float right_trigger() {
        return gamepad.right_trigger;
    }
    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                "LeftStickX" + Double.toString(left_stick_x()),
                "LeftStickY" + Double.toString(left_stick_y()),
                "RightStickX" + Double.toString(right_stick_x()),
                "RightStickY" + Double.toString(right_stick_y()),
        };
    }
}
