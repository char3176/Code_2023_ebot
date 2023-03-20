// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.IntakeCube;

public class IntakeRetractSpinot extends CommandBase {
  /** Creates a new IntakeRetractSpinot. */
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  public IntakeRetractSpinot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeCube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_IntakeCube.Retract();
    m_IntakeCube.spinIntake(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
