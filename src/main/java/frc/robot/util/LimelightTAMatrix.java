package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LimelightTAMatrix {
    private static InterpolatingDoubleTreeMap taMap = new InterpolatingDoubleTreeMap();

    public static void InitializeMatrix() {
        taMap.put(72.0, 19.25); // TA - Centimeters
        taMap.put(29.5, 25.0); 
        taMap.put(8.95, 50.0);
        taMap.put(4.5, 75.0);
        taMap.put(2.55, 100.0);
        taMap.put(1.7, 125.0);
        taMap.put(1.18, 150.0); // this is about the edge of its vision, always can improve quality of the vision by adding more values / keys
    }

    public static double get(double ta) {
        return taMap.get(ta);
    }
}
