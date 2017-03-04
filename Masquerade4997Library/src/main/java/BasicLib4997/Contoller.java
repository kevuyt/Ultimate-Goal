package BasicLib4997;

import BasicLib4997.MasqSensors.MasqClock;

/**
 * Created by Archish on 3/3/17.
 */

public class Contoller {
    MasqClock clock = new MasqClock();
    public Contoller(com.qualcomm.robotcore.hardware.Gamepad gamepad){
        this.gamepad  = gamepad;
        instance = this;
    }
    public static Contoller getDash(){
        return instance;
    }
    private static Contoller instance;
    private com.qualcomm.robotcore.hardware.Gamepad gamepad;
    boolean a() {
        return gamepad.a;
    }
    boolean x() {
        return gamepad.x;
    }
    boolean y() {
        return gamepad.y;
    }
    boolean b() {
        return gamepad.b;
    }
    boolean aIsDoubleTapped () {
        boolean doubleTapped = false;
         if (aPressedAndRealeased() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = a();
        }
        return doubleTapped;
    }
    boolean bIsDoubleTapped () {
        boolean doubleTapped = false;
        if (a() && !clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS)){
            doubleTapped = a();
        }
        return doubleTapped;
    }
    boolean xIsDoubleTapped () {
        return x() && clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS);
    }
    boolean yIsDoubleTapped () {
        return y() && clock.elapsedTime(0.5 ,MasqClock.Resolution.SECONDS);
    }
    boolean aPressedAndRealeased () {
        boolean pressed = false, realeaed = false;
        while (a()) {
            pressed = true;
        }
        while (!a()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }

}
