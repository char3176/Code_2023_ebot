// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Intake;
import team3176.robot.subsystems.superstructure.Superstructure;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class autoScoreConeHigh extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  Intake m_Intake = Intake.getInstance();
  Superstructure m_Superstructure = Superstructure.getInstance();
  Double CarryDeadband = 5.0;
  Double currentArmPosition;
  Double kArmPoopUpperLimit, kArmPoopLowerLimit, kArmCarryUpperLimit, kArmCarryLowerLimit;

  public autoScoreConeHigh() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
    addRequirements(m_Intake);
    addRequirements(m_Arm);
    addRequirements(m_Superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.extendAndFreeSpin();
    m_Superstructure.prepareScoreHigh();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    new WaitCommand(2);
    m_Claw.scoreGamePiece();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PoopCube End");
    m_Claw.idle();
    m_Intake.Retract();
    m_Intake.spinVelocityPercent(0);
    m_Superstructure.prepareCarry();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("PoopCube IsFinished");
    if (m_Claw.getLinebreakOne() == false || m_Claw.getLinebreakTwo() == false) {
      return true;
    } else{
      return false;
    }
  }
}
