// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeCubeIOTalon implements IntakeCubeIO{
    private TalonFX rollermotor;
    private TalonSRX conveyor;
    private DigitalInput linebreak;
    private DoubleSolenoid piston;


    IntakeCubeIOTalon()
    {
        rollermotor = new TalonFX(Hardwaremap.intake_CID);
        conveyor = new TalonSRX(Hardwaremap.conveyer_CID);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 6);
        linebreak = new DigitalInput(4);
    }

    @Override
    public void updateInputs(IntakeCubeIOInputs inputs) {
        inputs.appliedVoltsFX = rollermotor.getMotorOutputVoltage();
        inputs.appliedVoltsSRX = conveyor.getMotorOutputVoltage();
        inputs.isLinebreak = linebreak.get();
        inputs.tempCelciusFX = rollermotor.getTemperature();
        inputs.tempCelciusSRX = conveyor.getTemperature();
    }

    @Override
    public void setTalonFX(double pct) {
        rollermotor.configPeakOutputReverse(pct);
        rollermotor.configPeakOutputForward(pct);
        rollermotor.set(ControlMode.PercentOutput, pct);
    }

    @Override
    public void setTalonSRX(double pct) {
        conveyor.configPeakOutputReverse(pct);
        conveyor.configPeakOutputForward(pct);
        conveyor.set(ControlMode.PercentOutput, pct);
    }

    @Override
    public void setMode(int mode) {
        if (mode == 1)
        {
            rollermotor.setNeutralMode(NeutralMode.Coast);
        }
        else if (mode == 2)
        {
            rollermotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void Extend() {
        piston.set(Value.kForward);
    }

    @Override
    public void Retract() {
        piston.set(Value.kReverse);
    }
}