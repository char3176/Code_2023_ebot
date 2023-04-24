/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

import team3176.robot.subsystems.superstructure.IntakeConeIO;
import team3176.robot.subsystems.superstructure.IntakeConeIO.IntakeConeIOInputs;
import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.superstructure.Claw;

import team3176.robot.constants.Hardwaremap;

public class IntakeCone extends SubsystemBase {
  /** Creates a new IntakeCone. */
  

  private boolean isInIntake;
  private static IntakeCone instance;
  private Claw m_Claw;
  private final IntakeConeIO io;
  private final IntakeConeIOInputs inputs = new IntakeConeIOInputs();
  public IntakeCone(IntakeConeIO io) 
  {
    this.io = io;

    m_Claw = Claw.getInstance();

  }

  public void spit() 
  {
    //System.out.println("m_Claw.score()");
    inputs.spinVelocityPercent(1, 20);
  }

  public void idle() {
    //System.out.println("m_Claw.idle()");
    inputs.spinVelocityPercent(0, 0);
}

  public static IntakeCone getInstance(){
    if ( instance == null ) {
      instance = new IntakeCone(new IntakeConeIO() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("IntakeCone", inputs);
    Logger.getInstance().recordOutput("IntakeCone/Velocity", getVelocity());
    Logger.getInstance().recordOutput("IntakeCone/Linebreak", getIsLinebreakLogger());
    Logger.getInstance().recordOutput("IntakeCone/Extended", getIsExtended());
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    // SmartDashboard.putBoolean("isInIntake", isInIntake);
    SmartDashboard.putBoolean("ConeLinebreak", inputs.getLinebreak());
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public Command coneToClaw() {  
    return this.run(() ->  {spit();})
                .until(() -> this.m_Claw.inputs.getLinebreakTwo() == false)
                .andThen(new WaitCommand(0.5))
                .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle());
}

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public boolean getIsLinebreakLogger()
   {
    return inputs.isLinebreak;
   }

   public boolean getIsExtended()
   {
    return inputs.isextended;
   }

   public void runVoltage(double volts)
   {
    io.setVoltage(volts);
   }

   public void setVelocity(double velocity)
   {
    io.setVelocity(velocity);
   }

  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.inputs.Extend();
      this.inputs.spinVelocityPercent(.1, 25);
    }, () -> {
      this.inputs.Retract();
      this.inputs.spinVelocityPercent(0.0, 0);
    });
  }

  
  public Command extendAndFreeSpin() {
    return this.startEnd(() ->{
      this.inputs.Extend();
      this.inputs.setCoastMode();
    }, () -> {
      this.inputs.Retract();
      this.inputs.setBrakeMode();
    });
  }
}
