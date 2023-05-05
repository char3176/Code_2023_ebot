// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm.States;

/** Add your docs here. */
public class ArmIOSpark implements ArmIO {
    private CANSparkMax armController;
    private CANCoder armEncoder;
    private final PIDController m_turningPIDController;

    ArmIOSpark() {
        armController = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
        armController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
        armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);
        this.m_turningPIDController = new PIDController(SuperStructureConstants.ARM_kP, SuperStructureConstants.ARM_kI, SuperStructureConstants.ARM_kD);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs)
    {
        
    }

    @Override
    public void setSpark(double pct)
    {
        armController.set(pct);
    }

    @Override
    public void setSparkPIDPosMode(){
        this.armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
        this.armEncoder.configSensorDirection(true,100);

        this.m_turningPIDController.setTolerance(SuperStructureConstants.ARM_TOLERANCE);
        //this.m_turningPIDController.enableContinuousInput() 
        this.m_turningPIDController.reset();
        this.m_turningPIDController.setP(SuperStructureConstants.ARM_kP);
        this.m_turningPIDController.setI(SuperStructureConstants.ARM_kI);
        this.m_turningPIDController.setD(SuperStructureConstants.ARM_kD);
        //this.m_turningPIDController.enableContinuousInput(0, 360);
        this.armController.setOpenLoopRampRate(0.5);
        
        this.armController.burnFlash();
    }

    @Override
    public void setMode(int mode) {
        if (mode == 1)
        {
            armController.setIdleMode(IdleMode.kCoast);
        }
        else if (mode == 2)
        {
            armController.setIdleMode(IdleMode.kBrake);
        }
    }
}
