/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import team3176.robot.subsystems.superstructure.IntakeCubeIO.IntakeCubeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeCube extends SubsystemBase {
  /** Creates a new IntakeCube. */

  private static IntakeCube instance;
  private final IntakeCubeIO io;
  private final IntakeCubeIOInputs inputs = new IntakeCubeIOInputs();
  public IntakeCube(IntakeCubeIO io) 
  {
    this.io = io;
  }

  public static IntakeCube getInstance(){
    if ( instance == null ) {
      instance = new IntakeCube(new IntakeCubeIO() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    //Logger.getInstance().processInputs("IntakeCube", inputs);
    Logger.getInstance().recordOutput("IntakeCube/Velocity", inputs.velocity);
    Logger.getInstance().recordOutput("IntakeCube/Linebreak", inputs.isLinebreak);
    Logger.getInstance().recordOutput("IntakeCube/Extended", inputs.isextended);
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("CubeLinebreak", inputs.isLinebreak);
    // SmartDashboard.putBoolean("isInIntake", isInIntake);
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.io.Extend();
      this.io.setTalonFX(.1);
    }, () -> {
      this.io.Retract();
      this.io.setTalonFX(0.0);
    });
  }

  
  public Command extendAndFreeSpin() {
    return this.startEnd(() ->{
      this.io.Extend();
      this.io.setMode(1);
    }, () -> {
      this.io.Retract();
      this.io.setMode(2);
    });
  }
}