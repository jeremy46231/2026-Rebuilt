package frc.robot.util;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;

public class ActivityTimer {
    private double teleopStartTime;
    private boolean secondActiveAlliance;
    //= Utils.getCurrentTimeSeconds();

    public ActivityTimer() {
        teleopStartTime = Utils.getCurrentTimeSeconds();
    }

    void setAllianceOrdering(BooleanSupplier secondActiveAlliance) {
        this.secondActiveAlliance = secondActiveAlliance.getAsBoolean();
    }

    boolean getActiveHubAlliance() {

    } 
}
