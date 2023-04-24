// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


/** Template hardware interface for a closed loop subsystem. */
public interface IntakeConeIO{
  /** Contains all of the input data received from hardware. */
  public static class IntakeConeIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreak = true;
    // public double currentAmps;
    public double tempCelcius;
    public boolean isextended = false;

    private CANSparkMax rollermotor = new CANSparkMax(8, MotorType.kBrushless);
    private DoubleSolenoid pistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 3);
    private DigitalInput linebreak = new DigitalInput(3);



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreak", isLinebreak);
      table.put("AppliedVolts", appliedVolts);
      // table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("isextended", isextended);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreak = table.getBoolean("isLinebreak", isLinebreak);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      // currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      isextended = table.getBoolean("isextended", isextended);
    }

    public void spinVelocityPercent(double pct, int amps)
    { 
      rollermotor.set(-pct); 
      rollermotor.setSmartCurrentLimit(amps);
      velocity = pct;
      appliedVolts = amps;
    }

    public void setCoastMode() {rollermotor.setIdleMode(IdleMode.kCoast);}
    public void setBrakeMode() {rollermotor.setIdleMode(IdleMode.kBrake);} 
    public void Extend() {
      pistonOne.set(Value.kForward);
      this.isextended = true;
    }
  
    public void Retract() {
      pistonOne.set(Value.kReverse);
      this.isextended = false;
    }

    public boolean getLinebreak()
    {
      isLinebreak = linebreak.get();
      return linebreak.get();
    }

    public void getRollerTemp()
    {
      tempCelcius = rollermotor.getMotorTemperature();
    }
  }


  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeConeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}
}
