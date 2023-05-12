// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package team3176.robot.commands.superstructure.intakecone;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import team3176.robot.subsystems.superstructure.IntakeCone;
// import team3176.robot.subsystems.superstructure.Claw;
// import team3176.robot.constants.SuperStructureConstants;
// import team3176.robot.subsystems.superstructure.Arm;

// public class PoopCone extends CommandBase {
//   /** Creates a new PoopCone. */
//   IntakeCone m_IntakeCone = IntakeCone.getInstance();
//   Claw m_Claw = Claw.getInstance();
//   Arm m_Arm = Arm.getInstance();
//   public PoopCone() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_IntakeCone, m_Claw, m_Arm);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() 
//   {
//     m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS);
//     // m_Claw.setClawMotor(1, 20);
//     // m_IntakeCone.spinVelocityPercent(-.85, 25);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     m_IntakeCone.Extend();
//     m_Claw.setClawMotor(1, 20);
//     m_IntakeCone.spinVelocityPercent(-.85, 25);
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
