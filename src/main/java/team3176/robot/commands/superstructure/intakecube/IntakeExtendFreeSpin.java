// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.IntakeCube;

public class IntakeExtendFreeSpin extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  public IntakeExtendFreeSpin() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeCube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeCube.io.setMode(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_IntakeCube.io.Extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeCube.io.setMode(2);
    m_IntakeCube.io.Retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
