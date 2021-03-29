package MasqLibrary.MasqOdometry;

import com.qualcomm.robotcore.hardware.configuration.annotations.*;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by Keval Kataria on 3/20/2021
 */

@MotorType(ticksPerRev=1440, gearing=1, maxRPM=0, orientation=Rotation.CCW)
@DeviceProperties(xmlTag="E4T", name="E4T Encoder", builtIn = true)
public interface E4T {}