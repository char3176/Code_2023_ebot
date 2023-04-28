package team3176.robot.subsystems.drivetrain;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.SwervePodConstants2022;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import team3176.robot.util.God.*;


public class SwervePod {

    /** Class Object holding the Motor controller for Drive Motor on the SwervePod */
    /** Current value in radians of the azimuthEncoder's position */
    double azimuthEncoderRelPosition;
    double azimuthEncoderAbsPosition;
    double desired_optimized_azimuth_position;
    double velTicsPer100ms;
    boolean lastHasResetOccurred;

    /** Numerical identifier to differentiate between pods.
     *     For 4 Pods:  0 = FrontRight (FR),
     *                  1 = FrontLeft (FL),
     *                  2 = BackLeft (BL),
     *                  3 = BackRight (BR)
     */
    private int id;
    private String idString;
   
    /** Represents the value of the azimuthEncoder reading in radians when positioned with the positive Thrust vector of the Pod's Drive wheel pointing towards front of robot */
    //private double kAzimuthEncoderUnitsPerRevolution;

    private double lastEncoderPos;

    public int kSlotIdx_Azimuth, kPIDLoopIdx_Azimuth, kTimeoutMs_Azimuth,kSlotIdx_Thrust, kPIDLoopIdx_Thrust, kTimeoutMs_Thrust;

    public double podThrust, podAzimuth, podAbsAzimuth;

    private double kP_Azimuth;
    private double kI_Azimuth;
    private double kD_Azimuth;
    private double kRampRate_Azimuth = 0.0;



    private double turnOutput;
    private boolean isAutonSwerveControllerOptimizingAzimuthPos = false;

    private double PI = Math.PI;
    private final PIDController  turningPIDController;
    //private final ProfiledPIDController m_turningProfiledPIDController;
    //private ProfiledPIDController m_turningPIDController;

    private SwervePodIO io;
    private SwervePodIOInputsAutoLogged inputs = new SwervePodIOInputsAutoLogged();

    public SwervePod(int id, SwervePodIO io) {
        this.id = id;
        this.io = io;
        this.desired_optimized_azimuth_position = 0.0;
        


        this.kP_Azimuth = 0.006;
        this.kI_Azimuth = 0.0;
        this.kD_Azimuth = 0.0;


        
        
        turningPIDController = new PIDController(kP_Azimuth, kI_Azimuth, kD_Azimuth);
        turningPIDController.setTolerance(4);
        turningPIDController.enableContinuousInput(-180, 180);

        turningPIDController.reset();
        turningPIDController.setP(this.kP_Azimuth);
        turningPIDController.setI(this.kI_Azimuth);
        turningPIDController.setD(this.kD_Azimuth);
        
    }


    public void set(double speedMetersPerSecond, Rotation2d angle) {
        set_module(new SwerveModuleState(speedMetersPerSecond,angle));
    }

    /**
     *  alternative method for setting swervepod in line with WPILIB standard library
     * @param desiredState 
     */
    public SwerveModuleState set_module(SwerveModuleState desiredState) {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(this.id), inputs);
        
        this.azimuthEncoderAbsPosition = inputs.turnAbsolutePositionDegrees;
        SwerveModuleState desired_optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(this.azimuthEncoderAbsPosition));
        this.desired_optimized_azimuth_position = desired_optimized.angle.getDegrees();
        
        if (desiredState.speedMetersPerSecond > (-Math.pow(10,-10)) && desiredState.speedMetersPerSecond  < (Math.pow(10,-10))) {      
            this.turnOutput = turningPIDController.calculate(this.azimuthEncoderAbsPosition, this.lastEncoderPos);
        } else {
            this.turnOutput = turningPIDController.calculate(this.azimuthEncoderAbsPosition, desired_optimized.angle.getDegrees());
            this.lastEncoderPos = desired_optimized.angle.getDegrees(); 
        }
        // reduce output if the error is high
        desired_optimized.speedMetersPerSecond *= Math.abs(Math.cos(Units.degreesToRadians(turningPIDController.getPositionError())));
        //Logger.getInstance().recordOutput("Drive/Module" + Integer.toString(this.id) + "", id);
        io.setTurn(MathUtil.clamp(this.turnOutput, -0.4, 0.4));
        Logger.getInstance().recordOutput("Drive/Module" + Integer.toString(this.id) + "/error",turningPIDController.getPositionError());
        this.velTicsPer100ms = Units3176.mps2ums(desired_optimized.speedMetersPerSecond);
        io.setDrive(desired_optimized.speedMetersPerSecond);
        return desired_optimized;
    }   
    /*
     * odometry calls
     */
    public SwerveModulePosition getPosition() {
        double mps = Units.feetToMeters((DrivetrainConstants.WHEEL_DIAMETER_INCHES/12.0 * Math.PI)  *  inputs.drivePositionRad / (2*Math.PI));
        return new SwerveModulePosition(mps,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }


    public void goHome() {
        SwerveModuleState s = SwerveModuleState.optimize(new SwerveModuleState(0.0,Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(this.azimuthEncoderAbsPosition));

        // SmartDashboard.putNumber("P"+(this.id)+".optmizdazimuthAbsPos", optmizdAzimuthAbsPos);

        double turnOutput = turningPIDController.calculate(this.azimuthEncoderAbsPosition, s.angle.getDegrees());
        //double turnOutput = m_turningPIDController.calculate(this.azimuthEncoderPosition, this.podAzimuth);

        // SmartDashboard.putNumber("P"+(this.id)+".turnOutput", turnOutput);
        io.setTurn(turnOutput * SwervePodConstants2022.AZIMUTH_SPARKMAX_MAX_OUTPUTPERCENT);

        //this.azimuthPIDController.setReference(homePos, CANSparkMax.ControlType.kPosition);

    }


    public double getEncoderAbsPos() {
        return inputs.turnAbsolutePositionDegrees;
    } 
    
    public double getVelocity() {
        double wheelVelocityInFeetPerSecond = inputs.driveVelocityRadPerSec / (Math.PI *2) * DrivetrainConstants.WHEEL_DIAMETER_INCHES/12.0 * Math.PI;   
        double wheelVelocityInMetersPerSecond = Units3176.feetPerSecond2metersPerSecond(wheelVelocityInFeetPerSecond);
        return wheelVelocityInMetersPerSecond;
    }

    /**
     * Returns current Azimuth of pod in degrees, where 0 is straight forward.
     * @return
     */
    public double getAzimuth() {
        return inputs.turnAbsolutePositionDegrees; 
    }




    public void setThrustCoast() {
        io.setDriveBrakeMode(false);
    }


    public void setThrustBrake() {
        io.setDriveBrakeMode(true);
    }

    public double getAzimuthSetpoint() {
        return this.desired_optimized_azimuth_position;
    }
    public double getThrustSetpoint() {
        return this.velTicsPer100ms;
    }
    public double getThrustEncoderVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
    public double getAzimuthEncoderPosition() {
        return inputs.turnAbsolutePositionDegrees;
    }

    public void setupShuffleboard() {
        Shuffleboard.getTab(this.idString)
            .add(idString+"/podAzimuth_setpoint_angle",SwervePodConstants2022.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[id])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -3.16, "max", 3.16))
            .withSize(2,1)
            .withPosition(2,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kP_Azimuth", this.kP_Azimuth)
            .withSize(1,1)
            .withPosition(4,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kI_Azimuth", this.kI_Azimuth)
            .withSize(1,1)
            .withPosition(5,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kD_Azimuth", this.kD_Azimuth)
            .withSize(1,1)
            .withPosition(6,1)
            .getEntry();




     
        //public void calculate() {
        //  double distance = ...;
        //  distanceEntry.setDouble(distance);
        //}
    }

 // public double getRate(){
 // }
}
