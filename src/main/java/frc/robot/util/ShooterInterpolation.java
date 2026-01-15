package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {

    private final InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();

    /**
     * Represents <b>distance-to-angle</u> mapping used in shooter hood angle linear
     * interpolation.
     *
     * The more values defined, the more accurate stationary shots will be from
     * anywhere on the field. Define at least five values with <b>99% accuracy</b>.
     *
     * Table structure is defined below:
     * <table border="1">
     *   <tr><th>Distance (m)</th>      <th>Angle (deg)</th></tr>
     *   <tr><td>2.0</td>                   <td>x</td></tr>
     *   <tr><td>4.0</td>                   <td>x</td></tr>
     *   <tr><td>6.0</td>                   <td>x</td></tr>
     *   <tr><td>8.0</td>                   <td>x</td></tr>
     *   <tr><td>10.0</td>                  <td>x</td></tr>
     * </table>
     *
     */
    public ShooterInterpolation() {
        // TODO: Update dummy values with tested, accurate, true values.
        m_hoodMap.put(2.0, 60.0);
        m_hoodMap.put(4.0, 40.0);
        m_hoodMap.put(6.0, 20.0);
    }

    /**
     * Calculates the target hood angle based on distance to target.
     *
     * @param distanceMeters Distance to the target in meters.
     * @return Target hood angle in degrees (0 to 90).
     */
    public double getHoodAngle(double distanceMeters) {
        double angle = m_hoodMap.get(distanceMeters);
        // Clamp to physical limits 0-90 degrees
        return Math.max(0.0, Math.min(90.0, angle));
    }
}
