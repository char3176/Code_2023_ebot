// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

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
  public static class ClawIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreakOne = true;
    public boolean isLinebreakTwo = true;
    public boolean isLinebreakThree = true;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreakOne", isLinebreakOne);
      table.put("isLinebreakTwo", isLinebreakTwo);
      table.put("isLinebreakThree", isLinebreakThree);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreakOne = table.getBoolean("isLinebreakOne", isLinebreakOne);
      isLinebreakTwo = table.getBoolean("isLinebreakTwo", isLinebreakTwo);
      isLinebreakThree = table.getBoolean("isLinebreakThree", isLinebreakThree);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  public static class ClawIOHardware
  {
    private CANSparkMax claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);;
    private DigitalInput linebreakOne = new DigitalInput(0);
    private DigitalInput linebreakTwo = new DigitalInput(2);
    private DigitalInput linebreakThree = new DigitalInput(1);

    public void setClawVelocity(double percent) {claw.set(percent);}

    public void setClawAmps(int amps) {claw.setSmartCurrentLimit(amps);}

    public double getClawAmps() {return claw.getOutputCurrent();}

    public double getClawTemp() {return claw.getMotorTemperature();}

    public boolean getLinebreakOne() {return linebreakOne.get();}
    
    public boolean getLinebreakTwo() {return linebreakTwo.get();}

    public boolean getLinebreakThree() {return linebreakThree.get();}

  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}
}
