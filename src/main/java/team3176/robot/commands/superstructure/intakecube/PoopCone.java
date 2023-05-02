// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.IntakeCone;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;

public class PoopCone extends CommandBase {
  /** Creates a new IntakeCubeSpit. */
  IntakeCone m_IntakeCone = IntakeCone.getInstance();
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  public PoopCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeCone);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_IntakeCone.spit();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_IntakeCone.idle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
