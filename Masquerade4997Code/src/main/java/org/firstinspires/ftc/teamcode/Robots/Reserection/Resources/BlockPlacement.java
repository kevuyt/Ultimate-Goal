package org.firstinspires.ftc.teamcode.Robots.Reserection.Resources;

/**
 * Created by Archishmaan Peyyety on 1/4/19.
 * Project: MasqLib
 */
public enum BlockPlacement {
    LEFT("LEFT"),
    RIGHT("RIGHT"),
    CENTER("CENTER");
    private String string;
    BlockPlacement(String name){string = name;}
    @Override
    public String toString() {
        return string;
    }
}
