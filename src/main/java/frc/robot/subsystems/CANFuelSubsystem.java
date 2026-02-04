// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;

public class CANFuelSubsystem extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final SparkMax feederRoller;
  private final SparkMax launcherRoller;
  private final SparkMax intakeRoller;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder launcherEncoder;
  private final RelativeEncoder intakeEncoder;

  // Simulation objects
  private final SparkMaxSim feederSparkSim;
  private final SparkMaxSim launcherSparkSim;
  private final SparkMaxSim intakeSparkSim;
  private final DCMotor motorGearbox = DCMotor.getNEO(1);
  private final LinearSystem<N2, N1, N2> feederPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, FEEDER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim feederMotorSim = new DCMotorSim(feederPlant, motorGearbox);
  private final LinearSystem<N2, N1, N2> launcherPlant =
      LinearSystemId.createDCMotorSystem(motorGearbox, LAUNCHER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim launcherMotorSim = new DCMotorSim(launcherPlant, motorGearbox);
  private final DCMotorSim intakeMotorSim = new DCMotorSim(launcherPlant, motorGearbox);

  private static final String INTAKING_FEEDER_ROLLER_KEY = "Intaking feeder roller value";
  private static final String INTAKING_INTAKE_ROLLER_KEY = "Intaking intake roller value";
  private static final String LAUNCHING_FEEDER_ROLLER_KEY = "Launching feeder roller value";
  private static final String LAUNCHING_LAUNCHER_ROLLER_KEY = "Launching launcher roller value";
  private static final String LAUNCHING_INTAKE_ROLLER_KEY = "Launching intake roller value";
  private static final String SPINUP_FEEDER_ROLLER_KEY = "Spin-up feeder roller value";
  private SlewRateLimiter limiter;
  private double feederGoal = 0.0;
  private double launcherGoal = 0.0;
  private double intakeGoal = 0.0;
  private boolean intakeRunning = false;

  // For simulation
  private static final int MAX_BALLS = 12;
  private int ballCount = 8;
  private double launchDelay = TIME_BETWEEN_LAUNCHES;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

    limiter = new SlewRateLimiter(RATE_LIMIT);
    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederEncoder = feederRoller.getEncoder();
    launcherEncoder = launcherRoller.getEncoder();
    intakeEncoder = intakeRoller.getEncoder();

    launcherSparkSim = new SparkMaxSim(launcherRoller, motorGearbox);
    feederSparkSim = new SparkMaxSim(feederRoller, motorGearbox);
    intakeSparkSim = new SparkMaxSim(intakeRoller, motorGearbox);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashboard to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_INTAKE_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
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
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederGoal = SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    launcherGoal = 0.0;
    intakeGoal = SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederGoal = -1 * SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    launcherGoal = 0.0;
    intakeGoal = -1 * SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederGoal = SmartDashboard.getNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    launcherGoal =
        SmartDashboard.getNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
    intakeGoal = SmartDashboard.getNumber(LAUNCHING_INTAKE_ROLLER_KEY, LAUNCHING_INTAKE_VOLTAGE);
  }

  // A method to stop the rollers
  public void stop() {
    feederGoal = 0.0;
    launcherGoal = 0.0;
    intakeGoal = 0.0;
  }

  // A method to spin up the intake and launcher roller while spinning the feeder roller to keep the
  // balls out of the launcher
  public void spinUp() {
    feederGoal = SmartDashboard.getNumber(SPINUP_FEEDER_ROLLER_KEY, SPIN_UP_FEEDER_VOLTAGE);
    launcherGoal =
        SmartDashboard.getNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
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

  /** Method to check if the intake can accept more balls. */
  public boolean canIntakeBalls() {
    return intakeRunning && ballCount < MAX_BALLS;
  }

  /** Method to simulate adding a ball to the hopper. */
  public void addBallToHopper() {
    if (ballCount < MAX_BALLS) {
      ballCount++;
    }
  }

  /** A method to simulate launching a single ball from the hopper. */
  public void launchFuel() {
    if (ballCount == 0) {
      return;
    }
    ballCount--;
    Pose3d robot = new Pose3d(poseSupplier.get()).transformBy(ROBOT_TO_TURRET_TRANSFORM);
    Translation3d initialPosition = robot.getTranslation();
    LinearVelocity linearVel =
        MetersPerSecond.of(
            RPM.of(0.5 * getLauncherVelocity()).in(RadiansPerSecond) * FLYWHEEL_RADIUS.in(Meters));
    Angle angle = Degrees.of(120.0);
    Translation3d initialVelocity = launchVel(linearVel, angle);
    FuelSim.getInstance().spawnFuel(initialPosition, initialVelocity);
  }

  // A method to calculate the 3D launch velocity of the fuel based on the
  // robot's pose, robot speeds, launch speed, and launch elevation angle
  private Translation3d launchVel(LinearVelocity vel, Angle angle) {
    Pose3d robot = new Pose3d(poseSupplier.get()).transformBy(ROBOT_TO_TURRET_TRANSFORM);
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
    double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
    double xVel = horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
    double yVel = horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    return new Translation3d(xVel, yVel, verticalVel);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Use the slew rate limiter to ramp the feeder voltage to avoid sudden changes
    double feederVoltage = limiter.calculate(feederGoal);
    feederRoller.setVoltage(feederVoltage);
    launcherRoller.setVoltage(launcherGoal);
    intakeRoller.setVoltage(intakeGoal);
    if (intakeGoal > 0) {
      intakeRunning = true;
    } else if (intakeGoal == 0) {
      intakeRunning = false;
    }

    // Simulate the roller motors in simulation mode
    if (RobotBase.isSimulation()) {
      feederMotorSim.setInput(feederVoltage);
      feederMotorSim.update(0.020);
      feederSparkSim.iterate(feederMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

      launcherMotorSim.setInput(launcherGoal);
      launcherMotorSim.update(0.020);
      launcherSparkSim.iterate(launcherMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

      intakeMotorSim.setInput(intakeGoal);
      intakeMotorSim.update(0.020);
      intakeSparkSim.iterate(intakeMotorSim.getAngularVelocityRPM(), 12.0, 0.02);

      // Launch fuel with a delay between launches
      if (launchDelay <= 0.0 && getLauncherVelocity() > 1000.0 && getFeederVelocity() > 1000.0) {
        launchFuel();
        launchDelay = TIME_BETWEEN_LAUNCHES;
      } else {
        launchDelay -= 0.02;
      }
    }

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

    SmartDashboard.putNumber("Ball Count", ballCount);
  }
}
