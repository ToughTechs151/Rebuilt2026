package frc.sim;

/** Constants utility class for the claw simulation. */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  /** Drivetrain simulation constants. */
  public static final class DriveSimConstants {
    private DriveSimConstants() {
      throw new IllegalStateException("DriveSim Utility Class");
    }

    /** Number of motors driving each side of the drivetrain. */
    public static final int NUM_MOTORS = 2;

    /** Linear velocity feedforward velocity gain (V/(m/s)). */
    public static final double KV_LINEAR = 2.0;

    /** Linear velocity feedforward acceleration gain (V/(m/s^2)). */
    public static final double KA_LINEAR = 0.2;

    /** Angular velocity feedforward velocity gain (V/(rad/s)). */
    public static final double KV_ANGULAR = 3.0;

    /** Angular velocity feedforward acceleration gain (V/(rad/s^2)). */
    public static final double KA_ANGULAR = 0.3;

    /** Gain to apply to voltage command to get realistic current (0-1). */
    public static final double VOLT_SCALE_FACTOR = 0.7;
  }
}
