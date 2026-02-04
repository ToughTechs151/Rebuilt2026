package frc.sim;

import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.CANFuelSubsystem;

/** Simulate the fuel subsystem DC motors and fuel handling. */
public class FuelSubsystemSim {

  private final CANFuelSubsystem fuelSubsystem;
  private final SparkMaxSim feederSparkSim;
  private final SparkMaxSim launcherSparkSim;
  private final SparkMaxSim intakeSparkSim;
  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;
  private final SparkMax intakeMotor;

  private final DCMotor motorGearbox = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> feederPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, FEEDER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim feederMotorSim = new DCMotorSim(feederPlant, motorGearbox);

  private final LinearSystem<N2, N1, N2> launcherPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, LAUNCHER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim launcherMotorSim = new DCMotorSim(launcherPlant, motorGearbox);

  private final LinearSystem<N2, N1, N2> intakePlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, INTAKE_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim intakeMotorSim = new DCMotorSim(intakePlant, motorGearbox);

  /**
   * Create a new FuelSubsystemSim.
   *
   * @param fuelSubsystemToSimulate the CANFuelSubsystem to simulate
   */
  public FuelSubsystemSim(CANFuelSubsystem fuelSubsystemToSimulate) {

    fuelSubsystem = fuelSubsystemToSimulate;

    launcherMotor = fuelSubsystem.getLauncherMotor();
    feederMotor = fuelSubsystem.getFeederMotor();
    intakeMotor = fuelSubsystem.getIntakeMotor();

    launcherSparkSim = new SparkMaxSim(launcherMotor, motorGearbox);
    feederSparkSim = new SparkMaxSim(feederMotor, motorGearbox);
    intakeSparkSim = new SparkMaxSim(intakeMotor, motorGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {
    feederMotorSim.setInput(feederMotor.getAppliedOutput() * feederMotor.getBusVoltage());
    feederMotorSim.update(0.020);
    feederSparkSim.iterate(feederMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

    launcherMotorSim.setInput(launcherMotor.getAppliedOutput() * launcherMotor.getBusVoltage());
    launcherMotorSim.update(0.020);
    launcherSparkSim.iterate(launcherMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

    intakeMotorSim.setInput(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    intakeMotorSim.update(0.020);
    intakeSparkSim.iterate(intakeMotorSim.getAngularVelocityRPM(), 12.0, 0.02);
  }
}
