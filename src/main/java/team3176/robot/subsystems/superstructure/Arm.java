package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Arm extends SubsystemBase {
    private static Arm instance;
    private CANSparkMax armController;
    private CANCoder armEncoder;
    
    private final PIDController m_turningPIDController;


    private Arm() {
        armController = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
        armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);
        this.m_turningPIDController = new PIDController(SuperStructureConstants.ARM_kP, SuperStructureConstants.ARM_kI, SuperStructureConstants.ARM_kD);

        setArmPidPosMode();
    }

    private void setArmPidPosMode() {
        this.armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
        this.armEncoder.configSensorDirection(true,100);

        this.m_turningPIDController.setTolerance(SuperStructureConstants.ARM_TOLERANCE);
        //this.m_turningPIDController.enableContinuousInput() 
        this.m_turningPIDController.reset();
        this.m_turningPIDController.setP(SuperStructureConstants.ARM_kP);
        this.m_turningPIDController.setI(SuperStructureConstants.ARM_kI);
        this.m_turningPIDController.setD(SuperStructureConstants.ARM_kD);

        this.armController.setOpenLoopRampRate(0.5);
        
        this.armController.burnFlash();
    }

    public static Arm getInstance() {
        if (instance == null){instance = new Arm();}
        return instance;
    }

    private void armAnalogUp() {
        armController.set(SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    private void armAnalogDown() {
        armController.set(-SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    private void idle() {
        armController.set(0.0);
    }

    /*
     * Commands
     */
    public Command armAnalogUpCommand() {
        return this.startEnd(() -> armAnalogUp(), () -> idle());
    }
    public Command armAnalogDownCommand() {
        return this.startEnd(() -> armAnalogDown(), () -> idle());
    }
}
