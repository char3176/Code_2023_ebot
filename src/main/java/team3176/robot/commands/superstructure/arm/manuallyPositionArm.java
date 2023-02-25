// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class manuallyPositionArm extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  private Arm m_Arm = Arm.getInstance();
  private DoubleSupplier analogInput;
  private Double analogInputDeadband;

  public manuallyPositionArm(DoubleSupplier analogInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
    this.analogInput = analogInput;
    this.analogInputDeadband = 0.01;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double updatedAnalogInput = this.analogInput.getAsDouble(); 
    if (updatedAnalogInput > (0 + analogInputDeadband)) {
      m_Arm.armAnalogUpCommand();
    } 

    if (updatedAnalogInput < (0 - analogInputDeadband)) {
      m_Arm.armAnalogDownCommand();
    }
   
    //if ((updatedAnalogInput < (0 + analogInputDeadband)) && (updatedAnalogInput > (0 + analogInputDeadband))) {
    //  m_Arm.
    //}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Double updatedAnalogInput = this.analogInput.getAsDouble(); 
    if ((updatedAnalogInput < (0 + analogInputDeadband)) && (updatedAnalogInput > (0 - analogInputDeadband))) {
      return true;
    } else 
      return false;
  }
}
