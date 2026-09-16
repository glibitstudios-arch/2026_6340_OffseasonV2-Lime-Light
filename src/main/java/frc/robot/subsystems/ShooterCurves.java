package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.LauncherConstants;

public class ShooterCurves {

    public static double getRotationSpeed(double errorDeg) {
        double abs = Math.abs(errorDeg);
        double speed = abs < 1.0 ? 0.0 : abs < 10.0 ? map(abs, 1, 10, 0.2, 1.0) : map(abs, 10, 30, 1.0, 3.5);
        return Math.copySign(Math.min(speed, 3.5), errorDeg);
    }

    public static double getShooterVelocity(double dist) { 
        return calc(dist, LauncherConstants.NEAR_SHOOTER_VELOCITY, LauncherConstants.MID_SHOOTER_VELOCITY, LauncherConstants.FAR_SHOOTER_VELOCITY); 
    }
    
    public static double getHoodPosition(double dist) { 
        return calc(dist, LauncherConstants.NEAR_HOOD_POS, LauncherConstants.MID_HOOD_POS, LauncherConstants.FAR_HOOD_POS); 
    }

    private static double calc(double x, double y1, double y2, double y3) {
        x = MathUtil.clamp(x, LauncherConstants.NEAR_DISTANCE, LauncherConstants.FAR_DISTANCE);
        double x1 = LauncherConstants.NEAR_DISTANCE, x2 = LauncherConstants.MID_DISTANCE, x3 = LauncherConstants.FAR_DISTANCE;
        return y1*((x-x2)*(x-x3))/((x1-x2)*(x1-x3)) + y2*((x-x1)*(x-x3))/((x2-x1)*(x2-x3)) + y3*((x-x1)*(x-x2))/((x3-x1)*(x3-x2));
    }

    private static double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}