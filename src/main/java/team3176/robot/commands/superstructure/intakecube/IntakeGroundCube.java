// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import team3176.robot.subsystems.superstructure.IntakeCube;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;

public class IntakeGroundCube extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  public IntakeGroundCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeCube, m_Claw, m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Claw.setCurrentGamePiece(GamePiece.CUBE);
    m_Arm.armSetPositionBlocking(SuperStructureConstants.ARM_ZERO_POS);
    m_IntakeCube.io.Extend();
    m_IntakeCube.io.setTalonFX(-.85);
    m_Claw.intake();
    m_IntakeCube.io.setTalonSRX(-0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (!m_IntakeCube.inputs.isLinebreak)
    {
      m_IntakeCube.io.Retract();
    }
    m_Claw.intake();
    m_IntakeCube.io.setTalonSRX(-0.6);
    m_IntakeCube.io.setTalonFX(-.85);
    if (!m_Claw.inputs.isLinebreakOne)
    {
      m_Arm.armSetPosition(SuperStructureConstants.ARM_CARRY_POS);
    } else {m_Arm.armSetPosition(SuperStructureConstants.ARM_CATCH_POS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //m_IntakeCube.Retract();
    //new WaitCommand(0.1);
    m_IntakeCube.io.setTalonFX(0);
    m_IntakeCube.io.setTalonSRX(0);
    m_Claw.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_Claw.inputs.isLinebreakOne;
  }
}
