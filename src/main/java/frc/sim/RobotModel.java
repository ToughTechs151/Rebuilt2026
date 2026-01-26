package frc.sim;

/* PDP sim code poached from https://github.com/RobotCasserole1736/TheBestSwerve2021 */

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import java.util.Random;

/**
 * Robot simulation model for WPILib simulation support.
 *
 * <p>Provides realistic simulation of robot mechanics including elevator, claw, and roller systems.
 * Models battery voltage drops based on subsystem current draw and updates PDP simulation values.
 * Works in conjunction with subsystem classes to provide accurate simulation behavior.
 */
public class RobotModel {

  PDPSim simpdp;

  // Define subsystem simulation objects

  Random random = new Random(); // NOSONAR(java:S2245)

  private final boolean isReal;
  static final double QUIESCENT_CURRENT_DRAW_A = 2.0; // Misc electronics
  static final double BATTERY_NOMINAL_VOLTAGE = 13.2; // Nicely charged battery
  static final double BATTERY_NOMINAL_RESISTANCE = 0.010; // average battery + cabling
  double currentDrawA = QUIESCENT_CURRENT_DRAW_A;
  double batteryVoltageV = BATTERY_NOMINAL_VOLTAGE;

  /**
   * Create robot simulation. Does nothing if not running a simulation. Called from Robot.java as a
   * class field.
   *
   * @param robot the Robot instance
   */
  public RobotModel(Robot robot) {
    if (RobotBase.isSimulation()) {
      isReal = false;
    } else {
      isReal = true;
      return;
    }

    // Initialize subsystem simulations

    simpdp = new PDPSim(robot.getRobotContainer().getPdp());
    reset();
  }

  /** Update the simulation model. Call from simulationPeriodic method in robot.java. */
  public void update() {
    if (isReal) {
      return;
    }

    // Update subsystem simulations

    // Simulate battery voltage drop based on total simulated current
    // Get current draws from subsystems

    double[] simCurrents = {0}; // Insert simulated currents from subsystems here

    double unloadedVoltage = batteryVoltageV * 0.98 + ((random.nextDouble() / 10) - 0.05);
    double loadedVoltage =
        BatterySim.calculateLoadedBatteryVoltage(
            unloadedVoltage, BATTERY_NOMINAL_RESISTANCE, simCurrents);
    RoboRioSim.setVInVoltage(loadedVoltage);

    simpdp.setVoltage(loadedVoltage);
    simpdp.setCurrent(0, currentDrawA + random.nextDouble());
    // Set PDP currents from subsystems here
    simpdp.setTemperature(26.5);
  }

  /** Reset the simulation data. */
  public final void reset() {
    if (isReal) {
      return;
    }
    simpdp.resetData();
    RoboRioSim.resetData();
  }
}
