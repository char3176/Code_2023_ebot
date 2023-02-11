// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX rollermotor = new TalonFX(20);
  private DoubleSolenoid piston;
  private boolean isExtended;
  private static Intake instance;
  public Intake() 
  {
    piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
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

  public static Intake getInstance(){
    if ( instance == null ) {
      instance = new Intake();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
