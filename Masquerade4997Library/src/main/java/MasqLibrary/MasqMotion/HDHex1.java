package MasqLibrary.MasqMotion;

import com.qualcomm.robotcore.hardware.configuration.annotations.*;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by Keval Kataria on 3/23/2021
 */

@MotorType(ticksPerRev=28, gearing=1, maxRPM=6000, orientation= Rotation.CCW)
@DeviceProperties(xmlTag="HDHex1", name="HD Hex 1:1", builtIn = true)
public interface HDHex1 {}