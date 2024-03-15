package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

public class NavX {

    private static final AHRS navX = new AHRS(I2C.Port.kMXP);

    public static AHRS getNavX() {
        return navX;
    }
}
