// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.TunableNumber;

public class CANFuelSubsystem extends SubsystemBase {
  private final SwerveSubsystem drive;
  private final SparkMax feederRoller;
  private final SparkMax launcherRoller;
  private final SparkMax intakeRoller;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder launcherEncoder;
  private final RelativeEncoder intakeEncoder;

  private static final String INTAKING_FEEDER_ROLLER_KEY = "Intaking feeder roller value";
  private static final String INTAKING_INTAKE_ROLLER_KEY = "Intaking intake roller value";
  private static final String LAUNCHING_FEEDER_ROLLER_KEY = "Launching feeder roller value";
  private static final String LAUNCHING_INTAKE_ROLLER_KEY = "Launching intake roller value";
  private static final String SPINUP_FEEDER_ROLLER_KEY = "Spin-up feeder roller value";
  private SlewRateLimiter limiter;
  private double feederGoal = 0.0;
  private double launcherGoal = 0.0;
  private boolean launcherEnabled = false;
  private double intakeGoal = 0.0;

  private double pidOutput = 0.0;
  private double newFeedforward = 0;

  // Setup tunable numbers and controllers for the motor.
  private TunableNumber proportionalGain =
      new TunableNumber("Launcher Kp", LAUNCHER_KP_VOLTS_PER_RPM);
  private TunableNumber staticGain = new TunableNumber("Motor Ks", LAUNCHER_KS_VOLTS);
  private TunableNumber velocityGain = new TunableNumber("Launcher Kv", LAUNCHER_KV_VOLTS_PER_RPM);
  private TunableNumber accelerationGain =
      new TunableNumber("Launcher Ka", LAUNCHER_KA_VOLTS_PER_RPM2);
  private TunableNumber launcherRPM = new TunableNumber("Launcher Speed RPM", LAUNCHER_SPEED_RPM);

  private PIDController launcherController = new PIDController(proportionalGain.get(), 0.0, 0.0);

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(staticGain.get(), velocityGain.get(), accelerationGain.get());

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem(SwerveSubsystem drive) {
    this.drive = drive;

    limiter = new SlewRateLimiter(RATE_LIMIT);
    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederEncoder = feederRoller.getEncoder();
    launcherEncoder = launcherRoller.getEncoder();
    intakeEncoder = intakeRoller.getEncoder();

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashboard to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_INTAKE_ROLLER_KEY, LAUNCHING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(SPINUP_FEEDER_ROLLER_KEY, SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherRoller.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the intake roller, set a current limit, set
    // the motor to inverted so that positive values are used for intaking,
    // and apply the config to the controller
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederGoal = SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    launcherGoal = 0.0;
    launcherController.setSetpoint(launcherGoal);
    intakeGoal = SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederGoal = -1 * SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    launcherGoal = 0.0;
    launcherController.setSetpoint(launcherGoal);
    intakeGoal = -1 * SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    launcherGoal = launcherRPM.get();
    launcherController.setSetpoint(launcherGoal);
    intakeGoal = SmartDashboard.getNumber(LAUNCHING_INTAKE_ROLLER_KEY, LAUNCHING_INTAKE_VOLTAGE);
  }

  // A method to stop the rollers
  public void stop() {
    launcherEnabled = false;
    feederGoal = 0.0;
    launcherGoal = 0.0;
    launcherController.setSetpoint(launcherGoal);
    intakeGoal = 0.0;
  }

  // A method to spin up the intake and launcher roller while spinning the feeder
  // roller to keep the balls out of the launcher
  public void spinUp() {
    loadPidfTunableNumbers();
    launcherEnabled = true;
    launcherGoal = launcherRPM.get();
    launcherController.setSetpoint(launcherGoal);
    intakeGoal = SmartDashboard.getNumber(LAUNCHING_INTAKE_ROLLER_KEY, LAUNCHING_INTAKE_VOLTAGE);
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(this::spinUp);
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(this::launch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate the the motor command by adding the PID controller output and
    // feedforward to run the motor at the desired speed. Store the individual
    // values for logging.
    if (launcherEnabled) {
      pidOutput = launcherController.calculate(launcherEncoder.getVelocity());
      newFeedforward = feedforward.calculate(launcherController.getSetpoint());
      launcherRoller.setVoltage(pidOutput + newFeedforward);
      if (launcherEncoder.getVelocity() < launcherGoal * 0.9) {
        feederGoal = SmartDashboard.getNumber(SPINUP_FEEDER_ROLLER_KEY, SPIN_UP_FEEDER_VOLTAGE);
      } else {
        feederGoal =
            SmartDashboard.getNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
      }
    } else {
      launcherController.reset();
      pidOutput = 0.0;
      newFeedforward = 0.0;
      launcherRoller.setVoltage(0.0);
    }

    // Use the slew rate limiter to ramp the feeder voltage to avoid sudden changes
    double feederVoltage = limiter.calculate(feederGoal);
    feederRoller.setVoltage(feederVoltage);
    intakeRoller.setVoltage(intakeGoal);

    // Update SmartDashboard values for monitoring
    SmartDashboard.putNumber("Feeder Goal", feederGoal);
    SmartDashboard.putNumber("Feeder Set Voltage", feederVoltage);
    SmartDashboard.putNumber("Launcher Goal", launcherGoal);
    SmartDashboard.putNumber("Intake Goal", intakeGoal);

    SmartDashboard.putNumber("LauncherCurrent", launcherRoller.getOutputCurrent());
    SmartDashboard.putNumber("IntakeCurrent", intakeRoller.getOutputCurrent());
    SmartDashboard.putNumber(
        "LauncherVoltage", launcherRoller.getAppliedOutput() * launcherRoller.getBusVoltage());
    SmartDashboard.putNumber(
        "IntakeVoltage", intakeRoller.getAppliedOutput() * intakeRoller.getBusVoltage());
    SmartDashboard.putNumber("LauncherVelocity", launcherEncoder.getVelocity());
    SmartDashboard.putNumber("IntakeVelocity", intakeEncoder.getVelocity());

    SmartDashboard.putNumber("FeederCurrent", feederRoller.getOutputCurrent());
    SmartDashboard.putNumber(
        "FeederVoltage", feederRoller.getAppliedOutput() * feederRoller.getBusVoltage());
    SmartDashboard.putNumber("FeederVelocity", feederEncoder.getVelocity());

    SmartDashboard.putNumber("Launcher PID", pidOutput);
    SmartDashboard.putNumber("Launcher Feedforward", newFeedforward);
    SmartDashboard.putBoolean("Launcher Enabled", launcherEnabled);
  }

  /**
   * Load PIDF values that can be tuned at runtime. This should only be called when the controller
   * is disabled - for example from enable().
   */
  private void loadPidfTunableNumbers() {

    // Read tunable values for PID controller
    launcherController.setP(proportionalGain.get());

    // Read tunable values for Feedforward and create a new instance
    feedforward =
        new SimpleMotorFeedforward(staticGain.get(), velocityGain.get(), accelerationGain.get());
  }

  // Functions for simulation purposes

  /** Returns the feeder motor for simulation. */
  public SparkMax getFeederMotor() {
    return feederRoller;
  }

  /** Returns the intake motor for simulation. */
  public SparkMax getIntakeMotor() {
    return intakeRoller;
  }

  /** Returns the launcher motor for simulation. */
  public SparkMax getLauncherMotor() {
    return launcherRoller;
  }

  public double getLauncherVelocity() {
    return launcherEncoder.getVelocity();
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }
}
