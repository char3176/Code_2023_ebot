// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecone;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.IntakeCone;
import team3176.robot.subsystems.superstructure.Claw;

public class IntakeConeExtendSpin extends CommandBase {
  /** Creates a new IntakeConeExtend. */
  IntakeCone m_IntakeCone;
  Claw m_Claw;
  Timer continueRunningTimer;
  public IntakeConeExtendSpin() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeCone = IntakeCone.getInstance();
    m_Claw = Claw.getInstance();
    continueRunningTimer = new Timer();
    addRequirements(m_IntakeCone, m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_IntakeCone.Extend();
    m_IntakeCone.spinVelocityPercent(.85, 25);
    if (m_IntakeCone.getLinebreak())
    {
      continueRunningTimer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    while (m_Claw.getLinebreakThree() == true)
    {
      m_IntakeCone.spinVelocityPercent(-.14 * 2, 5);
    }
    m_IntakeCone.spinVelocityPercent(1, 20);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return continueRunningTimer.get() > 5;
  }
}
