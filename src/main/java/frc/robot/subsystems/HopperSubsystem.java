// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.TunableNumber;

/**
 * The {@code ArmSubsystem} class is a subsystem that controls the movement of an hopper using a
 * Profiled PID Controller. It uses a CANSparkMax motor and a RelativeEncoder to measure the
 * hopper's position. The class provides methods to move the hopper to a specific position, hold the
 * hopper at the current position, and shift the hopper's position up or down by a fixed increment.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of HopperSubsystem
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * HopperSubsystem hopperSubsystem = new HopperSubsystem(motor);
 *
 * // Move the hopper to a specific position
 * Command moveToPositionCommand = hopperSubsystem.moveToPosition(90.0);
 * moveToPositionCommand.schedule();
 *
 * // Hold the hopper at the current position
 * Command holdPositionCommand = hopperSubsystem.holdPosition();
 * holdPositionCommand.schedule();
 *
 * // Shift the hopper's position up by a fixed increment
 * Command shiftUpCommand = hopperSubsystem.shiftUp();
 * shiftUpCommand.schedule();
 *
 * // Shift the hopper's position down by a fixed increment
 * Command shiftDownCommand = hopperSubsystem.shiftDown();
 * shiftDownCommand.schedule();
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the movement of an hopper using a Profiled PID Controller
 *   - Move the hopper to a specific position
 *   - Hold the hopper at the current position
 *   - Shift the hopper's position up or down by a fixed increment
 * - Methods:
 *   - {@code periodic()}: Published telemetry with information about the hopper's state.
 *   - {@code useOutput()}: Generates the motor command using the PID controller and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the hopper to a new position.
 *   - {@code holdPosition()}: Returns a Command that holds the hopper at the last goal position.
 *   - {@code shiftUp()}: Returns a Command that shifts the hopper's position up by a fixed increment.
 *   - {@code shiftDown()}: Returns a Command that shifts the hopper's position down by a fixed
 *     increment.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the hopper has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the hopper.
 *   - {@code disable()}: Disables the PID control of the hopper.
 *   - {@code getMeasurement()}: Returns the hopper position for PID control and logging.
 *   - {@code getVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 * - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the hopper.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the hopper's
 *     position.
 *   - {@code private ProfiledPIDController hopperController}: The PID controller used to control the
 *     hopper's movement.
 *   - {@code private HopperFeedforward feedforward}: The feedforward controller used to calculate the
 *     motor output.
 *   - {@code private double output}: The output of the PID controller.
 *   - {@code private TrapezoidProfile.State setpoint}: The setpoint of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean hopperEnabled}: A flag indicating whether the hopper is enabled.
 *   - {@code private double voltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class HopperSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the hopper subsystem. */
  public static class Hardware {
    SparkMax motor;
    RelativeEncoder encoder;
    AbsoluteEncoder absoluteEncoder;

    public Hardware(SparkMax motor, RelativeEncoder encoder, AbsoluteEncoder absoluteEncoder) {
      this.motor = motor;
      this.encoder = encoder;
      this.absoluteEncoder = absoluteEncoder;
    }
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private ProfiledPIDController hopperController =
      new ProfiledPIDController(
          HopperConstants.HOPPER_KP,
          0,
          0,
          new TrapezoidProfile.Constraints(
              HopperConstants.HOPPER_MAX_VELOCITY_RAD_PER_SEC,
              HopperConstants.HOPPER_MAX_ACCELERATION_RAD_PER_SEC2));

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          HopperConstants.HOPPER_KS,
          HopperConstants.HOPPER_KG,
          HopperConstants.HOPPER_KV_VOLTS_PER_RAD_PER_SEC,
          0.0); // Acceleration is not used in this implementation

  private double output = 0.0;
  private TrapezoidProfile.State setpoint = new State();
  private double newFeedforward = 0;
  private boolean hopperEnabled;
  private double voltageCommand = 0.0;
  private double hopperOffset;
  private boolean absoluteEncoderValid = true;
  private int absoluteEncoderFreezeCount = 0;
  private double absoluteEncoderLastValue = 0.0;

  // Setup tunable numbers for the hopper.
  private TunableNumber kp = new TunableNumber("HopperKP", HopperConstants.HOPPER_KP);
  private TunableNumber ks = new TunableNumber("HopperKS", HopperConstants.HOPPER_KS);
  private TunableNumber kg = new TunableNumber("HopperKG", HopperConstants.HOPPER_KG);
  private TunableNumber kv =
      new TunableNumber("HopperKV", HopperConstants.HOPPER_KV_VOLTS_PER_RAD_PER_SEC);
  private TunableNumber maxVelocity =
      new TunableNumber("HopperMaxVelocity", HopperConstants.HOPPER_MAX_VELOCITY_RAD_PER_SEC);
  private TunableNumber maxAcceleration =
      new TunableNumber(
          "HopperMaxAcceleration", HopperConstants.HOPPER_MAX_ACCELERATION_RAD_PER_SEC2);

  /** Create a new HopperSubsystem controlled by a Profiled PID COntroller . */
  public HopperSubsystem(Hardware hopperHardware) {
    this.motor = hopperHardware.motor;
    this.encoder = hopperHardware.encoder;
    this.absoluteEncoder = hopperHardware.absoluteEncoder;

    initializeHopper();
  }

  private void initializeHopper() {

    initMotor();

    hopperOffset =
        RobotBase.isReal()
            ? getAbsoluteAngle()
            : Units.radiansToDegrees(HopperConstants.HOPPER_OFFSET_RADS);
    DataLogManager.log("Hopper Offset Data: " + hopperOffset);
    // Set tolerances that will be used to determine when the hopper is at the goal position.
    hopperController.setTolerance(
        Constants.HopperConstants.POSITION_TOLERANCE, Constants.HopperConstants.VELOCITY_TOLERANCE);

    disable();

    // Add buttons for the dashboard
    SmartDashboard.putData(
        "Hopper Brake Mode",
        new InstantCommand(() -> setBrakeMode(true))
            .ignoringDisable(true)
            .withName("Hopper Brake Mode"));

    SmartDashboard.putData(
        "Hopper Coast Mode",
        new InstantCommand(() -> setBrakeMode(false))
            .ignoringDisable(true)
            .withName("Hopper Coast Mode"));

    SmartDashboard.putData(
        "Hopper Reset Position",
        new InstantCommand(this::resetEncoder)
            .ignoringDisable(true)
            .withName("Hopper Reset Position"));

    SmartDashboard.putData(shiftUp());
    SmartDashboard.putData(shiftDown());

    setDefaultCommand(runOnce(this::disable).andThen(run(() -> {})).withName("Idle"));
  }

  private void initMotor() {
    motorConfig.smartCurrentLimit(HopperConstants.CURRENT_LIMIT);
    motorConfig.inverted(HopperConstants.INVERTED);

    // Setup the encoder scale factors. Since this is a relative encoder,
    // hopper position will only be correct if the hopper is in the starting rest position when
    // the subsystem is constructed.
    motorConfig
        .encoder
        .positionConversionFactor(HopperConstants.HOPPER_RAD_PER_ENCODER_ROTATION)
        .velocityConversionFactor(HopperConstants.RPM_TO_RAD_PER_SEC);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.clearFaults();
    encoder.setPosition(0);

    // Configure the motor to use EMF braking when idle.
    setBrakeMode(true);
    DataLogManager.log("Hopper motor firmware version:" + motor.getFirmwareString());
  }

  /**
   * Initialize hardware devices for the hopper subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    SparkMax motor = new SparkMax(HopperConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();

    return new Hardware(motor, encoder, absoluteEncoder);
  }

  @Override
  public void periodic() {

    checkAbsoluteEncoder();

    SmartDashboard.putBoolean("Hopper/Enabled", hopperEnabled);
    SmartDashboard.putNumber(
        "Hopper/Goal", Units.radiansToDegrees(hopperController.getGoal().position));
    SmartDashboard.putNumber("Hopper/Angle", getRelativeAngle());
    SmartDashboard.putNumber("Hopper/Absolute Angle", getAbsoluteAngle());
    SmartDashboard.putNumber("Hopper/Combined Angle", Math.toDegrees(getMeasurement()));
    SmartDashboard.putNumber("Hopper/Encoder Delta", (getRelativeAngle() - getAbsoluteAngle()));
    SmartDashboard.putNumber("Hopper/Voltage", voltageCommand);
    SmartDashboard.putNumber("Hopper/Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Hopper/Temp", motor.getMotorTemperature());
    SmartDashboard.putNumber("Hopper/SetPt Pos", Units.radiansToDegrees(setpoint.position));
    SmartDashboard.putBoolean("Hopper/Encoder Valid", absoluteEncoderValid);
    SmartDashboard.putBoolean("Hopper/At Goal", atGoalPosition());

    if (Constants.SD_SHOW_HOPPER_EXTENDED_LOGGING_DATA) {
      SmartDashboard.putNumber("Hopper/Feedforward", newFeedforward);
      SmartDashboard.putNumber("Hopper/PID output", output);
      SmartDashboard.putNumber("Hopper/SetPt Vel", Units.radiansToDegrees(setpoint.velocity));
      SmartDashboard.putNumber("Hopper/Velocity", Units.radiansToDegrees(encoder.getVelocity()));
    }
  }

  /** Generate the motor command using the PID controller and feedforward. */
  public void useOutput() {
    if (hopperEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      output = hopperController.calculate(getMeasurement());
      setpoint = hopperController.getSetpoint();

      // Calculate the feedforward to move the hopper at the desired velocity and offset
      // the effect of gravity at the desired position. Voltage for acceleration is not
      // used.
      newFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;

    } else {
      // If the hopper isn't enabled, set the motor command to 0. In this state the hopper
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      output = 0;
      newFeedforward = 0;
      voltageCommand = 0;
    }
    motor.setVoltage(voltageCommand);
  }

  /** Returns a Command that moves the hopper to a new position. */
  public Command moveToPosition(double goal) {
    return new FunctionalCommand(
        () -> setGoalPosition(goal),
        this::useOutput,
        interrupted -> {},
        this::atGoalPosition,
        this);
  }

  // Command to move hopper to extended position if not already there, or to retracted position if
  // already extended.
  public Command extendHopperCommand() {
    return moveToPosition(Constants.HopperConstants.HOPPER_EXTENDED_RADS).withName("Extend Hopper");
  }

  public Command retractHopperCommand() {
    return moveToPosition(Constants.HopperConstants.HOPPER_RETRACTED_RADS)
        .withName("Retract Hopper");
  }

  /**
   * Returns a Command that holds the hopper at the last goal position using the PID Controller
   * driving the motor.
   */
  public Command holdPosition() {
    return run(this::useOutput).withName("Hopper: Hold Position");
  }

  /** Abort Command will set the hopper position to the goal position in any restricted areas. */
  public Command abortCommand() {
    return new InstantCommand(
        () -> {
          setGoalPosition(getMeasurement());
          hopperController.reset(getMeasurement());
        });
  }

  /** Returns a Command that shifts hopper position up by a fixed increment. */
  public Command shiftUp() {
    return runOnce(
            () ->
                setGoalPosition(
                    hopperController.getGoal().position + Constants.HopperConstants.POS_INCREMENT))
        .andThen(run(this::useOutput))
        .until(this::atGoalPosition)
        .withName("Hopper: Shift Position Up");
  }

  /** Returns a Command that shifts hopper position down by a fixed increment. */
  public Command shiftDown() {
    return runOnce(
            () ->
                setGoalPosition(
                    hopperController.getGoal().position - Constants.HopperConstants.POS_INCREMENT))
        .andThen(run(this::useOutput))
        .until(this::atGoalPosition)
        .withName("Hopper: Shift Position Down");
  }

  /**
   * Set the goal state for the subsystem, limited to allowable range. Goal velocity is set to zero.
   * The ProfiledPIDController drives the hopper to this position and holds it there.
   */
  private void setGoalPosition(double goal) {
    hopperController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.HopperConstants.MIN_ANGLE_RADS,
                Constants.HopperConstants.MAX_ANGLE_RADS),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the hopper has reached the goal position and velocity is within limits. */
  public boolean atGoalPosition() {
    return hopperController.atGoal();
  }

  /**
   * Sets up the PID controller to move the hopper to the defined goal position and hold at that
   * position. Preferences for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!hopperEnabled) {
      loadTunableNumbers();
      setDefaultCommand(holdPosition());

      // Reset the PID controller to clear any previous state
      hopperController.reset(getMeasurement());
      hopperEnabled = true;

      DataLogManager.log(
          "Hopper Enabled - kP="
              + hopperController.getP()
              + " kI="
              + hopperController.getI()
              + " kD="
              + hopperController.getD()
              + " PosGoal="
              + Units.radiansToDegrees(hopperController.getGoal().position)
              + " CurPos="
              + Units.radiansToDegrees(getMeasurement()));
    }
  }

  /**
   * Disables the PID control of the hopper. Sets motor output to zero. NOTE: In this state the
   * hopper will move until it hits the stop. Using EMF braking mode with motor will slow this
   * movement.
   */
  public void disable() {

    // Clear the enabled flag and call useOutput to zero the motor command
    hopperEnabled = false;
    useOutput();
    setDefaultCommand(run(() -> {}).withName("Idle"));

    DataLogManager.log(
        "Hopper Disabled CurPos="
            + Units.radiansToDegrees(getMeasurement())
            + " CurVel="
            + Units.radiansToDegrees(encoder.getVelocity()));
  }

  /** Returns the relative hopper angle using the built in encoder. The units are in degrees */
  public double getRelativeAngle() {
    return Math.toDegrees(encoder.getPosition() + HopperConstants.HOPPER_OFFSET_RADS);
  }

  /**
   * Returns the hopper position for PID control and logging (Units are Radians from horizontal).
   */
  public double getMeasurement() {
    if (absoluteEncoderValid) {
      return Math.toRadians(getAbsoluteAngle());
    } else {
      // If the absolute encoder is not valid, use the relative encoder.
      return encoder.getPosition() + HopperConstants.HOPPER_OFFSET_RADS;
    }
  }

  /** Returns the absolute hopper angle. The units are in degrees */
  public double getAbsoluteAngle() {
    // Add the offset from the 0 point of the encoder and wrap to within +/-180
    double angle = absoluteEncoder.getPosition() * 360 - HopperConstants.ABSOLUTE_OFFSET_DEGREES;
    if (angle >= 180.0) {
      angle -= 360.0;
    } else if (angle < -180.0) {
      angle += 360;
    }
    return angle;
  }

  // Check the absolute encoder for freeze condition. Due to noise a normally functioning encoder
  // will change value continually.
  private void checkAbsoluteEncoder() {

    if (RobotBase.isReal()) {

      Double absoluteEncoderValue = absoluteEncoder.getPosition();
      if (absoluteEncoderValue.equals(absoluteEncoderLastValue)) {
        absoluteEncoderFreezeCount++;
        if (absoluteEncoderFreezeCount > 20) {
          absoluteEncoderValid = false;
        }
      } else {
        absoluteEncoderFreezeCount = 0;
      }
      absoluteEncoderLastValue = absoluteEncoderValue;
    }
  }

  /** Returns the Motor Commanded Voltage. */
  public double getVoltageCommand() {
    return voltageCommand;
  }

  /** Returns the motor for simulation. */
  public SparkMax getMotor() {
    return motor;
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    if (enableBrake) {
      DataLogManager.log("Hopper motor set to brake mode");
      brakeConfig.idleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Hopper motor set to coast mode");
      brakeConfig.idleMode(IdleMode.kCoast);
    }
    motor.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Reset the encoder to the start (zero) position. This should only be done when the hopper is
   * resting against the back stop. This command doesn't work in simulation.
   */
  public void resetEncoder() {
    DataLogManager.log("Hopper encoder reset");
    encoder.setPosition(0);
  }

  /**
   * Checks if the hopper is enabled or not.
   *
   * @return boolean hopperEnabled
   */
  public boolean isEnabled() {
    return hopperEnabled;
  }

  /**
   * Load values that can be tuned at runtime. This should only be called when the controller is
   * disabled - for example from enable().
   */
  private void loadTunableNumbers() {

    // Read Preferences for PID controller
    hopperController.setP(kp.get());

    // Read Preferences for Trapezoid Profile and update
    hopperController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Read Preferences for Feedforward and create a new instance
    feedforward = new ArmFeedforward(ks.get(), kg.get(), kv.get(), 0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
