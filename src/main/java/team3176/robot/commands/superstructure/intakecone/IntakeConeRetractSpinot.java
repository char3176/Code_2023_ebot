// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package team3176.robot.commands.superstructure.intakecone;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import team3176.robot.subsystems.superstructure.IntakeCone;

// public class IntakeConeRetractSpinot extends CommandBase {
//   /** Creates a new IntakeConeRetractSpinot. */
//   IntakeCone m_IntakeCone;
//   public IntakeConeRetractSpinot() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_IntakeCone = IntakeCone.getInstance();
//     addRequirements(m_IntakeCone);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     m_IntakeCone.Retract();
//     m_IntakeCone.spinVelocityPercent(0, 0);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
