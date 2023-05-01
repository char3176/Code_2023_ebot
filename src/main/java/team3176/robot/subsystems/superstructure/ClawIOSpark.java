// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;
import team3176.robot.constants.Hardwaremap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/** Template hardware interface for a closed loop subsystem. */
public class ClawIOSpark implements ClawIO{
    private CANSparkMax clawSpark;
    private DigitalInput linebreakOne;
    private DigitalInput linebreakTwo;
    private DigitalInput linebreakThree;

    ClawIOSpark()
    {
        clawSpark = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
        linebreakOne = new DigitalInput(0);
        linebreakTwo = new DigitalInput(2);
        linebreakThree = new DigitalInput(1);
        clawSpark.setSmartCurrentLimit(20);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.appliedVolts = clawSpark.getAppliedOutput() * clawSpark.getBusVoltage();
        inputs.isLinebreakOne = linebreakOne.get();
        inputs.isLinebreakTwo = linebreakTwo.get();
        inputs.isLinebreakThree = linebreakThree.get();
        inputs.tempCelcius = clawSpark.getMotorTemperature();
    }

    @Override
    public void set(double percent, int amps) {
        clawSpark.set(percent);
        clawSpark.setSmartCurrentLimit(amps);
    }
}