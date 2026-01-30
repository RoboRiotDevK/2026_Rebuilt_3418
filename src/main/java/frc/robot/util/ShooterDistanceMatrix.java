package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterDistanceMatrix {
    private static InterpolatingDoubleTreeMap matrix = new InterpolatingDoubleTreeMap();

    public static void InitializeMatrix() {
        matrix.put(0.0, 0.0);
    }

    public static double get(double val) {
        return matrix.get(val);
    }
}
