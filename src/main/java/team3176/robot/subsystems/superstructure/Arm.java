package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Arm extends SubsystemBase {
    private static Arm instance;
    private CANSparkMax arm;
    private Arm() {
        arm = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
    }
    public static Arm getInstance() {
        if (instance == null){instance = new Arm();}
        return instance;
    }

    private void armUp() {
        arm.set(SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    private void armDown() {
        arm.set(-SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    private void idle() {
        arm.set(0.0);
    }

    /*
     * Commands
     */
    public Command armUpCommand() {
        return this.startEnd(() -> armUp(), () -> idle());
    }
    public Command armDownCommand() {
        return this.startEnd(() -> armDown(), () -> idle());
    }
}
