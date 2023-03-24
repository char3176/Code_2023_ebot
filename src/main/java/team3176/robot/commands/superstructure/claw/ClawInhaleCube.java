// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.IntakeCube;
import team3176.robot.subsystems.RobotState.GamePiece;
import team3176.robot.subsystems.RobotState;
public class ClawInhaleCube extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  Timer continueRunningTimer = new Timer();
  public ClawInhaleCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState.getInstance().setWantedGamePiece(GamePiece.CUBE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //m_Claw.intake();
    m_Claw.setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
    if(m_Claw.getLinebreakCube()) {
    continueRunningTimer.restart();
    }
    m_Claw.determineGamePiece();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Claw.hold();
    m_Claw.setClawMotor(SuperStructureConstants.CLAW_HOLD_POWER * SuperStructureConstants.CLAW_HOLD_CONE_FACTOR,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
    m_IntakeCube.spinConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return continueRunningTimer.get() > 1.0;
  }
}
