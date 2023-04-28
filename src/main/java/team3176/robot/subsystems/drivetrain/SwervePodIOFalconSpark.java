package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import team3176.robot.constants.SwervePodConstants2022;
import team3176.robot.constants.SwervePodHardwareID;

public class SwervePodIOFalconSpark implements SwervePodIO{
    private CANSparkMax turnSparkMax;
    private TalonFX thrustFalcon;
    private CANCoder azimuthEncoder;
    public SwervePodIOFalconSpark(SwervePodHardwareID id,int sparkMaxID) {
        turnSparkMax = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
        thrustFalcon = new TalonFX(id.THRUST_CID);
        //reset the motor controllers
        thrustFalcon.configFactoryDefault();
        turnSparkMax.restoreFactoryDefaults();

        thrustFalcon.configClosedloopRamp(0.5);   
        thrustFalcon.setInverted(false);
        thrustFalcon.config_kP(0, 0.03);
        thrustFalcon.config_kI(0, 0.0);
        thrustFalcon.config_kD(0, 0.0);
        thrustFalcon.config_kF(0, 0.045);
        
        turnSparkMax.setOpenLoopRampRate(0.5);
        turnSparkMax.setSmartCurrentLimit(20);
        turnSparkMax.setInverted(true);

        azimuthEncoder = new CANCoder(id.CANCODER_CID);
        azimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        azimuthEncoder.configMagnetOffset(id.OFFSET);
        azimuthEncoder.configSensorDirection(true,100);
        
    }
    public void updateInputs(SwervePodIOInputs inputs) {
        inputs.drivePositionEncoder = thrustFalcon.getSelectedSensorPosition();
        inputs.driveVelocityTics = thrustFalcon.getSelectedSensorVelocity();
        inputs.driveAppliedVolts = thrustFalcon.getMotorOutputVoltage();
        inputs.driveCurrentAmps = new double[] {thrustFalcon.getStatorCurrent()};
        inputs.driveTempCelcius = new double[] {thrustFalcon.getTemperature()};

        inputs.turnAbsolutePositionDegrees = azimuthEncoder.getAbsolutePosition();
        
        inputs.turnVelocityRPM = turnSparkMax.getEncoder().getVelocity();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    }

    @Override
    public void setDrive(double velTicsPer100ms) {
        thrustFalcon.set(TalonFXControlMode.Velocity, velTicsPer100ms);
    }

    /** Run the turn motor at the specified voltage. */
    public void setTurn(double volts) {
        turnSparkMax.set(volts);
    }

    /** Enable or disable brake mode on the drive motor. */
    public void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public void setTurnBrakeMode(boolean enable) {}
}
