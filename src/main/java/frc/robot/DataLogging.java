package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** The DataLogging class contains all the logic for using telemetry. */
public class DataLogging { // NOSONAR(java:S6548)

  private DoubleLogEntry loopTime;
  private double startTime;
  private boolean everBrownout = false;
  private PowerDistribution pdp;

  private double totalAmpHours = 0.0;
  private double totalWattHours = 0.0;

  private double lastTimestamp = 0.0;
  private boolean wasDisabled = true;

  private DoubleLogEntry ampHoursLog;
  private DoubleLogEntry wattHoursLog;

  private DataLogging() {
    // Starts recording to data log
    DataLogManager.start();
    final DataLog log = DataLogManager.getLog();

    // Record both DS control and joystick data.
    DriverStation.startDataLog(DataLogManager.getLog(), Constants.LOG_JOYSTICK_DATA);

    DataLogManager.log(String.format("Brownout Voltage: %f", RobotController.getBrownoutVoltage()));

    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish

    StringLogEntry commandLog = new StringLogEntry(log, "/command/event");
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> commandLog.append("Command initialized:" + command.getName()));

    if (Constants.COMMAND_EXECUTE_LOG) {
      CommandScheduler.getInstance()
          .onCommandExecute(command -> commandLog.append("Command execute:" + command.getName()));
    }

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> commandLog.append("Command interrupted:" + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> commandLog.append("Command finished:" + command.getName()));
    commandLog.append("Opened command log");

    loopTime = new DoubleLogEntry(log, "/robot/LoopTime");

    ampHoursLog = new DoubleLogEntry(log, "/battery/AmpHours");
    wattHoursLog = new DoubleLogEntry(log, "/battery/WattHours");

    lastTimestamp = Timer.getFPGATimestamp();
  }

  private static class InstanceHolder {
    private static final DataLogging instance = new DataLogging();
  }

  /**
   * Gets the datalogging Singleton object.
   *
   * @return DataLogging
   */
  public static DataLogging getInstance() {
    return InstanceHolder.instance;
  }

  /**
   * Called from robot.java immediately after the robotContainer is created.
   *
   * @param robotContainer The robotContainer just constructed.
   */
  public void dataLogRobotContainerInit(RobotContainer robotContainer) {

    // Add hardware sendables here
    pdp = robotContainer.getPdp();
    SmartDashboard.putData("PDP", pdp);

    // Log configuration info here
    DataLogManager.log(String.format("PDP Can ID: %d", pdp.getModule()));
  }

  /**
   * Runs at each loop slice. Should be called in the robotPeriodic method in Robot.java as the last
   * thing in the method.
   *
   * <pre>{@code
   * //must be at end
   * datalog.periodic();
   * }</pre>
   */
  public void periodic() {

    if (RobotController.isBrownedOut()) {
      everBrownout = true;
    }

    if (Constants.LOOP_TIMING_LOG) {
      loopTime.append(Timer.getFPGATimestamp() - startTime);
    }

    // Reset accumulators at start of enable (new match)
    if (DriverStation.isEnabled() && wasDisabled) {
      totalAmpHours = 0.0;
      totalWattHours = 0.0;
    }
    wasDisabled = DriverStation.isDisabled();

    // Time delta calculation
    double currentTime = Timer.getFPGATimestamp();
    double deltaTimeSeconds = currentTime - lastTimestamp;
    lastTimestamp = currentTime;
    double deltaTimeHours = deltaTimeSeconds / 3600.0;

    double totalCurrent = pdp.getTotalCurrent();
    double voltage = pdp.getVoltage();
    double power = totalCurrent * voltage;

    // Integrate
    totalAmpHours += totalCurrent * deltaTimeHours;
    totalWattHours += power * deltaTimeHours;

    // Log to DataLog
    ampHoursLog.append(totalAmpHours);
    wattHoursLog.append(totalWattHours);

    // Send to SmartDashboard
    SmartDashboard.putNumber("Battery/AmpHoursUsed", totalAmpHours);
    SmartDashboard.putNumber("Battery/WattHoursUsed", totalWattHours);

    SmartDashboard.putNumber("Robot/Batt Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Robot/Brown Out", RobotController.isBrownedOut());
    SmartDashboard.putBoolean("Robot/Ever Browned Out", this.getEverBrownOut());
    SmartDashboard.putNumber("Robot/PDP Temperature", pdp.getTemperature());

    HubTracker.updateTelemetry();
  }

  /** Records the current timestamp for loop timing calculations. */
  public void startLoopTime() {
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Returns whether the robot has ever experienced a brownout since startup.
   *
   * @return true if the robot has browned out at least once, false otherwise
   */
  public final boolean getEverBrownOut() {
    return this.everBrownout;
  }
}
