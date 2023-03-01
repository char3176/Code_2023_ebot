package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Arm extends SubsystemBase {
    private static final double MAX_ENCODER_ANGLE_VALUE = SuperStructureConstants.ARM_HIGH_POS;
    private static final double MIN_ENCODER_ANGLE_VALUE = SuperStructureConstants.ARM_ZERO_POS;
    private static Arm instance;
    private CANSparkMax armController;
    private CANCoder armEncoder;
    private double armEncoderAbsPosition;    
    private double lastEncoderPos;
    private final PIDController m_turningPIDController;
    private int counter;
    public enum States {OPEN_LOOP,CLOSED_LOOP};
    private States currentState = States.OPEN_LOOP;
    private double armSetpointAngleRaw = SuperStructureConstants.ARM_ZERO_POS;

    private Arm() {
        armController = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
        armController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
        armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);
        this.m_turningPIDController = new PIDController(SuperStructureConstants.ARM_kP, SuperStructureConstants.ARM_kI, SuperStructureConstants.ARM_kD);
        SmartDashboard.putNumber("Arm_kp", SuperStructureConstants.ARM_kP);
        SmartDashboard.putNumber("Arm_Kg", SuperStructureConstants.ARM_kg);
        setArmPidPosMode();
    }

    public void setCoastMode() {
        armController.setIdleMode(IdleMode.kCoast);
    }

    public void setBrakeMode() {
        armController.setIdleMode(IdleMode.kBrake);
    } 

    private void setArmPidPosMode() {
        this.armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
        this.armEncoder.configSensorDirection(false,100);

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

    public static Arm getInstance() {
        if (instance == null){instance = new Arm();}
        return instance;
    }
    
    /**
     * 
     * @param desiredAngle in degrees in Encoder Frame
     */
    private void setPIDPosition(double desiredAngle) {
        //need to double check these values
        
        this.armEncoderAbsPosition = armEncoder.getAbsolutePosition();
        double physicsAngle = (this.armEncoderAbsPosition - SuperStructureConstants.ARM_CARRY_POS);
        
        //kg is the scalar representing the percent power needed to hold the arm at 90 degrees away from the robot
        double kg = SmartDashboard.getNumber("Arm_Kg", SuperStructureConstants.ARM_kg);
        // kp set as the fraction of control effort / error to cause control effort
        // for example .4 output is generated by a 40 degree error
        double kp = SmartDashboard.getNumber("Arm_kp", SuperStructureConstants.ARM_kP);
        m_turningPIDController.setP(kp);
        double feedForward = kg * physicsAngle/SuperStructureConstants.ARM_HIGH_POS;
        if (this.armEncoderAbsPosition < SuperStructureConstants.ARM_MID_POS + 10){
            feedForward =0.0;
        }
        double turnOutput = m_turningPIDController.calculate(this.armEncoderAbsPosition, desiredAngle);
        turnOutput = MathUtil.clamp(turnOutput,-0.2,0.2);
        armController.set(turnOutput + feedForward);
        SmartDashboard.putNumber("Arm_Output", turnOutput + feedForward);
        SmartDashboard.putNumber("Arm Feed Forward", feedForward);
    }
    public void armAnalogUp() {
        this.currentState = States.OPEN_LOOP;
        armController.set(SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    public void armAnalogDown() {
        this.currentState = States.OPEN_LOOP;
        armController.set(-SuperStructureConstants.ARM_OUTPUT_POWER);
        System.out.println("Arm Analog Down"); 
        System.out.println("Arm_Abs_Position: " + armEncoder.getAbsolutePosition()); 
        System.out.println("Arm_Rel_Position: " + armEncoder.getPosition());
    }
    public void idle() {
        armController.set(0.0);
    }
    public void fineTune(double delta) {
        this.currentState = States.CLOSED_LOOP;
        this.armSetpointAngleRaw += delta * 0.5;
        
    }
    private void nothing() {

    }
    public double getArmPosition() {
        return armEncoder.getAbsolutePosition();
    }
    public boolean isArmAtPosition() {
        return Math.abs(this.armEncoder.getAbsolutePosition() - this.armSetpointAngleRaw) < 7;
    }
    /**
     * to be used for trajectory following without disrupting other commands
     * @param setpointAngle
     */
    public void setAngleSetpoint(double setpointAngle) {
        this.currentState = States.CLOSED_LOOP;
        this.armSetpointAngleRaw = setpointAngle;
    }
    /*
     * Commands
     */
    public Command armSetPosition(double angleInDegrees) {
        return this.run(() -> setPIDPosition(angleInDegrees));
    }
    public Command armSetPositionBlocking(double angleInDegrees) {
        return new FunctionalCommand(() -> {
            this.currentState = States.CLOSED_LOOP;
            this.armSetpointAngleRaw = angleInDegrees;}, 
            ()-> this.nothing(), 
            (b) -> this.nothing(), 
            this::isArmAtPosition, 
            this);
    }
    public Command armSetPositionOnce(double angleInDegrees) {
        return this.runOnce(() -> {
            this.currentState = States.CLOSED_LOOP;
            this.armSetpointAngleRaw = angleInDegrees;});
    }
    public Command armFineTune(DoubleSupplier angleDeltaCommand) {
        return this.run(() -> fineTune(-angleDeltaCommand.getAsDouble()));
    }
    public Command armAnalogUpCommand() {
        return this.runEnd(() -> armAnalogUp(), () -> idle());
    }
    public Command armAnalogDownCommand() {
        return this.runEnd(() -> armAnalogDown(), () -> idle());
    }
    
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm_Position_Relative", armEncoder.getAbsolutePosition() - SuperStructureConstants.ARM_ZERO_POS);

        // if (counter == 50) {
        //     System.out.println("Arm_Position: " + armEncoder.getAbsolutePosition()); 
        //     counter = 0;
        // } else {
        //     counter = counter++;
        // }
        if(this.currentState == States.CLOSED_LOOP) {
            this.armSetpointAngleRaw = MathUtil.clamp(this.armSetpointAngleRaw, SuperStructureConstants.ARM_ZERO_POS, SuperStructureConstants.ARM_HIGH_POS);
            SmartDashboard.putNumber("Arm_Error", armEncoder.getAbsolutePosition()-this.armSetpointAngleRaw);
            setPIDPosition(armSetpointAngleRaw);
        }
    }
}
