package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class Claw extends SubsystemBase {
    private CANSparkMax claw;
    private DigitalInput linebreakOne;
    private DigitalInput linebreakTwo;
    private static Claw instance;
    public GamePiece currentGamePiece = GamePiece.NONE;
    private Claw() {
        instance = new Claw();
        claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
        linebreakOne = new DigitalInput(0);
        linebreakTwo = new DigitalInput(2);
    }
    public void setClawMotor(double percent, int amps) {
        claw.set(percent);
        claw.setSmartCurrentLimit(amps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", claw.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", claw.getMotorTemperature());
    }

    //states now implemented as functions
    
    private void intake() {
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    private void hold() {
        if(currentGamePiece == GamePiece.CONE) {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            
        } else {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }
    private void score() {
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    private void idle() {
        setClawMotor(0, 0);
    }

    public boolean getLinebreaks()
    {
        if (linebreakOne.get() == false || linebreakTwo.get() == false)
        {
            return false;
        }
        else 
        {
            return true;
        }
    }

    public static Claw getInstance()
    {
        return instance;
    }

    /**
     * to be called with a whileTrue trigger binding
     */
    public CommandBase intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {this.currentGamePiece = piece; intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue trigger binding
     */
    public Command scoreGamePiece() {
        return this.startEnd(() ->  {score(); this.currentGamePiece = GamePiece.NONE;},() -> idle());
    }

    //more examples of command composition and why its awesome!!
    public Command intakeCone() {
        return this.intakeGamePiece(GamePiece.CONE).until(this::getLinebreaks);
    }
    public Command intakeCube() {
        return this.intakeGamePiece(GamePiece.CUBE).until(this::getLinebreaks);
    }
}
