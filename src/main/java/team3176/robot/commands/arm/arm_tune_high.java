// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class arm_tune_high  extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  private Arm m_Arm = Arm.getInstance();
  private DoubleSupplier analogInput;
  private Double analogInputDeadband;
  private Timer timer = new Timer();

  public arm_tune_high() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      m_Arm.setPIDPosition(315);
  }
    //if ((updatedAnalogInput < (0 + analogInputDeadband)) && (updatedAnalogInput > (0 + analogInputDeadband))) {
    //  m_Arm.
    //}
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.idle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
