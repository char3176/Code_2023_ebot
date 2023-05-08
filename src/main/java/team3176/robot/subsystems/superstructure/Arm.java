package team3176.robot.subsystems.superstructure;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.ArmIO.ArmIOInputs;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static Arm instance;
    private final ArmIO io;
    private final ArmIOInputs inputs = new ArmIOInputs();  
    public enum States {OPEN_LOOP,CLOSED_LOOP};
    private States currentState = States.OPEN_LOOP;
    private double armSetpointAngleRaw = SuperStructureConstants.ARM_ZERO_POS;

    private Arm(ArmIO io) {
        this.io = io;
        io.setSparkPIDPosMode();
    }

    public static Arm getInstance() {
        if (instance == null){instance = new Arm(new ArmIO() {});}
        return instance;
    }

    public void armAnalogUp() {
        this.currentState = States.OPEN_LOOP;
        io.setSpark(SuperStructureConstants.ARM_OUTPUT_POWER);
    }

    public void armAnalogDown() {
        this.currentState = States.OPEN_LOOP;
        io.setSpark(-SuperStructureConstants.ARM_OUTPUT_POWER);
    }

    public void fineTune(double delta) {
        this.currentState = States.CLOSED_LOOP;
        this.armSetpointAngleRaw -= delta * 0.5;
        
    }
    private void nothing() {}

    public boolean isArmAtPosition() {
        return Math.abs(this.inputs.position - this.armSetpointAngleRaw) < 7;
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
        SmartDashboard.putNumber("armSetPosition",angleInDegrees);
        return this.run(() -> io.setPIDPosition(angleInDegrees, inputs));
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
        return this.run(() -> fineTune(angleDeltaCommand.getAsDouble()));
    }
    public Command armAnalogUpCommand() {
        return this.runEnd(() -> armAnalogUp(), () -> io.setSpark(0));
    }
    public Command armAnalogDownCommand() {
        return this.runEnd(() -> armAnalogDown(), () -> io.setSpark(0));
    }
    
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().recordOutput("Arm/Position", getPosition());

        SmartDashboard.putNumber("Arm_Position", inputs.position);
        SmartDashboard.putNumber("Arm_Position_Relative", inputs.position - SuperStructureConstants.ARM_ZERO_POS);

        // if (counter == 50) {
        //     System.out.println("Arm_Position: " + armEncoder.getAbsolutePosition()); 
        //     counter = 0;
        // } else {
        //     counter = counter++;
        // }
        if(this.currentState == States.CLOSED_LOOP) {
            this.armSetpointAngleRaw = MathUtil.clamp(this.armSetpointAngleRaw, SuperStructureConstants.ARM_ZERO_POS, SuperStructureConstants.ARM_HIGH_POS);
            SmartDashboard.putNumber("Arm_Error", inputs.position - this.armSetpointAngleRaw);
            io.setPIDPosition(armSetpointAngleRaw, inputs);
        }
    }

    public double getPosition()
    {
        return inputs.position;
    }
}
