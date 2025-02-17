package frc.lib.math;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio      Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts    Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double neoToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
     *                  RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
     *                      Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    // public static double neoToMeters(double positionCounts, double circumference,
    // double gearRatio){
    public static double neoToMeters(double positionCounts, double circumference, double gearRatio) {
        // return positionCounts * (circumference / (gearRatio * 42.0));
        return positionCounts / 42;
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToNeo(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 42.0));
    }

    /**
     * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS      Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference  Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters   Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
}