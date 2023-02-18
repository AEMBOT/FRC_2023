package frc.robot;

import static frc.robot.Constants.ArmConstants.*;

public class MiscellaneousFunctions {
    /**
     * Converts the angle of the arm into the necessary amount of belt released, in meters
     *
     * @param radians The angle of the arm, in radians. 0 = Pointing directly forward, π/2 = Pointing directly upward
     * @return The required belt distance, in meters.
     */
    public static double ArmAngleToDistance(double radians) {
        return Math.sqrt(Math.pow((pivotPointDistance + pivotRadius * Math.cos(radians)), 2) + Math.pow((pivotPointHeight + pivotRadius * Math.sin(radians)), 2));
    }

    public static double DistanceToArmAngle(double distance) {
        double radians = distance / (2 * Math.PI);
        double a = -pivotPointDistance * Math.cos(radians);
        double b = -pivotPointHeight * Math.sin(radians);
        double c = Math.sqrt(-Math.pow(pivotPointDistance, 2) * Math.pow(Math.sin(radians), 2) + 2 * pivotPointDistance * pivotPointHeight
                * Math.sin(radians) * Math.cos(radians) - Math.pow(pivotPointHeight, 2) * Math.pow(Math.cos(radians), 2) + Math.pow(pivotRadius, 2));
        return a + b + c;
    }


}
