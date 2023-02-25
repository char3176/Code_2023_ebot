// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Intake;
import team3176.robot.subsystems.superstructure.Superstructure;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class PoopCube extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  Intake m_Intake = Intake.getInstance();
  Superstructure m_Superstructure = Superstructure.getInstance();
  Double CarryDeadband = 5.0;
  Double currentArmPosition;
  Double kArmUpperLimit, kArmLowerLimit;

  public PoopCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.extendAndFreeSpin();
    currentArmPosition = m_Arm.getArmPosition();
    kArmLowerLimit = SuperStructureConstants.ARM_ZERO_POS - this.CarryDeadband;
    kArmUpperLimit = SuperStructureConstants.ARM_ZERO_POS + this.CarryDeadband;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Superstructure.preparePoop();
    if (m_Arm.getArmPosition() >= SuperStructureConstants.ARM_ZERO_POS) {
      m_Claw.scoreGamePiece();
    }
    if (m_Claw.getLinebreakOne() == true && m_Claw.getLinebreakTwo() == true) {
      m_Claw.idle();
      m_Superstructure.prepareCarry();
    }
    this.currentArmPosition = m_Arm.getArmPosition();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.Retract();
    m_Intake.spinVelocityPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentArmPosition >= kArmLowerLimit && currentArmPosition >= kArmUpperLimit) {
      return true;
    } else{
      return false;
    }
  }
}
