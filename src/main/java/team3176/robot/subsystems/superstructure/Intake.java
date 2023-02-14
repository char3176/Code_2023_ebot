// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX rollermotor = new TalonFX(20);
  private DoubleSolenoid piston;
  private DigitalInput linebreak;

  private boolean isExtended;
  private boolean isInIntake;
  private boolean isCone;
  private boolean isSquircle;
  private I2C.Port m_I2C = I2C.Port.kOnboard;
  private ColorSensorV3 m_ColorSensor = new ColorSensorV3(m_I2C);
  private static Intake instance;
  public Intake() 
  {
    piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    linebreak = new DigitalInput(0);

  }

  public void spinVelocityPercent(double pct) {
    rollermotor.set(ControlMode.PercentOutput, pct);
  }

  public void Extend() {
    piston.set(Value.kForward);
    this.isExtended = true;
  }

  public void Retract() {
    piston.set(Value.kReverse);
    this.isExtended = false;
  }

  public boolean isCone()
  {
    return isCone;
  }

  public boolean isSquircle()
  {
    return isSquircle;
  }

  public static Intake getInstance(){
    if ( instance == null ) {
      instance = new Intake();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_ColorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

    
    if ((0.35 <= detectedColor.red && detectedColor.red <= 0.379) && 
        (0.466 <= detectedColor.green && detectedColor.green <= 0.516) && 
        (0.083 <= detectedColor.blue && detectedColor.blue <= 0.188))
    {
      isCone = true;
      // System.out.println("TRUE");
    }
    else 
    {
      isCone = false;
      // System.out.println("FALSE");
    }
    SmartDashboard.putBoolean("isCone", isCone);
    
    if ((0.241 <= detectedColor.red && detectedColor.red <= 0.318) && 
        (0.382 <= detectedColor.green && detectedColor.green <= 0.458) && 
        (0.227 <= detectedColor.blue && detectedColor.blue <= 0.375))
    {
      isSquircle = true;
    }
    else 
    {
      isSquircle = false;
    }
    SmartDashboard.putBoolean("isSquircle", isSquircle);



    // Code stating if something is in the Intake
    if (m_ColorSensor.getProximity() <= 150)
    {
    if (linebreak.get() == false)
    {
      isInIntake = true;
      if (this.isExtended = true)
        {
          // Retract();
        }
      }
      else
      {
        isInIntake = false;
      }
    }
    SmartDashboard.putBoolean("isInIntake", isInIntake);
    SmartDashboard.putBoolean("isExtended", isExtended);
    SmartDashboard.putNumber("getProximity", m_ColorSensor.getProximity());

   }

  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.spinVelocityPercent(.3);
    }, () -> {
      this.Retract();
      this.spinVelocityPercent(0.0);
    });
  }
}
