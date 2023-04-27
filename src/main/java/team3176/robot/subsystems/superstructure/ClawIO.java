// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/** Template hardware interface for a closed loop subsystem. */
public interface ClawIO{
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClawIOInputs{
    public double appliedVolts = 0.0;
    public boolean isLinebreakOne = true;
    public boolean isLinebreakTwo = true;
    public boolean isLinebreakThree = true;
    // public double[] currentAmps = new double[] {};
    public double tempCelcius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  public default void set(double percent, int amps) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
