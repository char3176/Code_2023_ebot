// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.Timer;
import team3176.robot.subsystems.superstructure.IntakeCube;

public class IntakeExtendSpin extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  Timer continueRunningTimer = new Timer();
  public IntakeExtendSpin() {
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
    m_IntakeCube.Extend();
    m_IntakeCube.spinConveyor(0.4);
    m_IntakeCube.spinIntake(.85);
    if (m_IntakeCube.getLinebreak())
    {
      continueRunningTimer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_IntakeCube.Retract();
    new WaitCommand(0.5);
    m_IntakeCube.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return continueRunningTimer.get() > 0.5;
  }
}
