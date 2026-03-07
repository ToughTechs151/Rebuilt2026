// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.sim.Constants.HopperSim;

/** A robot hopper simulation based on a linear system model with Mech2d display. */
public class HopperModel implements AutoCloseable {

  private final HopperSubsystem hopperSubsystem;
  private double simHopperCurrent = 0.0;
  private SparkMaxSim sparkSimHopper;

  // The hopper gearbox represents a gearbox containing one motor.
  private final DCMotor hopperGearbox = DCMotor.getNEO(1);

  // This hopper sim represents an hopper that can rotate over the given mechanical range when
  // driven
  // by the motor under the effect of gravity.
  private final SingleJointedArmSim hopperSim =
      new SingleJointedArmSim(
          hopperGearbox,
          HopperConstants.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(HopperSim.HOPPER_LENGTH_METERS, HopperSim.HOPPER_MASS_KG),
          HopperSim.HOPPER_LENGTH_METERS,
          HopperConstants.MIN_ANGLE_RADS,
          HopperConstants.MAX_ANGLE_RADS,
          true,
          HopperSim.START_ANGLE_RADS,
          HopperSim.ENCODER_DISTANCE_PER_PULSE,
          0.0 // Add noise with a std-dev of 1 tick
          );

  private SparkAbsoluteEncoderSim absoluteEncoderSim;

  // Create a Mechanism2d visualization of the hopper.
  private final Mechanism2d mech2d = new Mechanism2d(1, 2);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Hopper Root", 0.5, 0.0);
  private final MechanismLigament2d hopperMech2d =
      mech2dRoot.append(
          new MechanismLigament2d(
              "Hopper",
              HopperSim.HOPPER_LENGTH_METERS,
              Units.radiansToDegrees(hopperSim.getAngleRads()) - 90,
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new HopperModel. */
  public HopperModel(HopperSubsystem hopperSubsystemToSimulate) {

    hopperSubsystem = hopperSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Hopper visualization, select Network Tables -> SmartDashboard -> Hopper Sim
    SmartDashboard.putData("Hopper Sim", mech2d);
  }

  /** Initialize the hopper simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax motors and methods to set values
    sparkSimHopper = new SparkMaxSim(hopperSubsystem.getMotor(), hopperGearbox);
    absoluteEncoderSim = sparkSimHopper.getAbsoluteEncoderSim();
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our hopper are doing
    // First, we set our "inputs" (voltages)
    hopperSim.setInput(hopperSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    hopperSim.update(0.020);

    // Finally, we run the spark simulations and save the current so it can be retrieved later.
    sparkSimHopper.iterate(hopperSim.getVelocityRadPerSec(), 12.0, 0.02);
    sparkSimHopper.setPosition(hopperSim.getAngleRads() - HopperConstants.HOPPER_OFFSET_RADS);
    absoluteEncoderSim.setPosition(
        Units.radiansToRotations(hopperSim.getAngleRads())
            + Units.degreesToRotations(HopperConstants.ABSOLUTE_OFFSET_DEGREES));

    // Update hopper visualization with angle
    hopperMech2d.setAngle(Units.radiansToDegrees(hopperSim.getAngleRads()) - 90);
  }

  /** Return the simulated hopper motor current. */
  public double getSimHopperCurrent() {
    return simHopperCurrent;
  }

  @Override
  public void close() {
    hopperMech2d.close();
  }
}
