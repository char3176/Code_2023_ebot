package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Superstructure extends SubsystemBase {
    
    private Superstructure instance;
    
    
    

    public static enum GamePiece {CUBE, CONE, NONE};
    

    private Superstructure() {
    }
    public Superstructure getInstance() {
        if (instance == null){instance = new Superstructure();}
        return instance;
    }
    
    
    public Command scoreMid() {
        return new WaitCommand(0);
    }
    public Command scoreHigh() {
        return new WaitCommand(0);
    }
    public Command intake(GamePiece p) {
        return new WaitCommand(0);
    }
    @Override
    public void periodic() {
        
        
    }
    
}
