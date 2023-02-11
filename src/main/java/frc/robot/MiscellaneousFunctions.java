package frc.robot;

import static frc.robot.Constants.ArmConstants.*;

public class MiscellaneousFunctions{
    /**
     * Converts the angle of the arm into the necessary amount of belt released, in meters
     * @param radians The angle of the arm, in radians. 0 = Pointing directly forward, π/2 = Pointing directly upward
     * @return The required belt distance, in meters.
     */
    public static double ArmAngleToDistance(double radians){
        return Math.sqrt(Math.pow((pivotPointDistance+pivotRadius * Math.cos(radians)),2) + Math.pow((pivotPointHeight+pivotRadius * Math.sin(radians)),2));
    }
}
