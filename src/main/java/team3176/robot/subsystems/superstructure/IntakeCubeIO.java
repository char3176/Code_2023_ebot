// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
@AutoLog
public interface IntakeCubeIO{
  /** Contains all of the input data received from hardware. */
  public static class IntakeCubeIOInputs{
    public double velocity = 0.0;
    public double appliedVoltsSRX = 0.0;
    public double appliedVoltsFX = 0.0;
    public boolean isLinebreak = true;
    public double currentAmpsSRX = 0.0;
    public double currentAmpsFX = 0.0;
    public double tempCelciusFX = 0.0;
    public double tempCelciusSRX = 0.0;
    public boolean isextended = false;
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeCubeIOInputs inputs) {}

  public default void setTalonFX(double pct) {}

  public default void setTalonSRX(double pct) {}

  public default void setMode(int mode) {}

  public default void Extend() {}
  public default void Retract() {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}
}
