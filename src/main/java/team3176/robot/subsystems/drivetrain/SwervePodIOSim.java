package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team3176.robot.Constants;
import team3176.robot.Robot;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.SwervePodHardwareID;

public class SwervePodIOSim implements SwervePodIO{
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 4.714, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNeo550(1), 70.0, 0.004);
    private PIDController drivePID = new PIDController(.03, 0, 0.0,.045);
    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double currentDriveSpeed = 0.0;
    public SwervePodIOSim() {
        
        
    }
    public void updateInputs(SwervePodIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);
        double angleDiffRad = Units.radiansToDegrees(turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < -180) {
        turnAbsolutePositionRad += 360;
        }
        while (turnAbsolutePositionRad > 180) {
        turnAbsolutePositionRad -= 360;
        }

        inputs.drivePositionRad =
            inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
        inputs.driveTempCelcius = new double[] {};
        currentDriveSpeed = driveSim.getAngularVelocityRadPerSec();

        inputs.turnAbsolutePositionDegrees = turnAbsolutePositionRad;
        inputs.turnVelocityRPM = turnSim.getAngularVelocityRPM();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
        inputs.turnTempCelcius = new double[] {};
        
    }

    @Override
    public void setDrive(double velMetersPerSecond) {
        double freeSpeedRadPerSec = 142.0;
        double driveSpeedError = velMetersPerSecond - (this.currentDriveSpeed / (2) * Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES)); 
        double voltage = 12 * velMetersPerSecond * 1.0/(freeSpeedRadPerSec / (2*Math.PI) * Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI);
        driveAppliedVolts = MathUtil.clamp(voltage + driveSpeedError*4,-12.0,12.0);
        if(DriverStation.isEnabled()) {
            driveSim.setInputVoltage(driveAppliedVolts);
        }else {
            driveSim.setInputVoltage(0.0);
        }
        
    }

    /** Run the turn motor at the specified voltage. */
    public void setTurn(double volts) {
        if(DriverStation.isEnabled()){
            turnAppliedVolts = MathUtil.clamp(volts * 12, -12.0, 12.0);
            turnSim.setInputVoltage(turnAppliedVolts);
        }else {
            turnSim.setInput(0.0);
        }
        
    }

    /** Enable or disable brake mode on the drive motor. */
    public void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public void setTurnBrakeMode(boolean enable) {}
}
