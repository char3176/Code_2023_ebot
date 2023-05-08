// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;
import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmIO{
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs{
    public double position = 0.0;
    public double appliedVolts = 0.0;
    //public double currentAmps = 0.0;
    public double tempCelcius = 0.0;
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setSpark(double pct) {}

  public default void setSparkPIDPosMode() {}
  public default void setPIDPosition(double desiredAngle, ArmIOInputs inputs) {}

  public default void setP(double kp) {}

  public default void setMode(int mode) {}
}

