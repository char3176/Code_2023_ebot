package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class Claw extends SubsystemBase {
    private CANSparkMax intake;
    public GamePiece currentGamePiece = GamePiece.NONE;
    private Claw() {
        intake = new CANSparkMax(Hardwaremap.intake_CID, MotorType.kBrushless);
    }
    public void setIntakeMotor(double percent, int amps) {
        intake.set(percent);
        intake.setSmartCurrentLimit(amps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
    }

    //states now implemented as functions
    
    private void intake() {
        if(currentGamePiece == GamePiece.CUBE) {
            setIntakeMotor(SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             setIntakeMotor(-SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
         }
    }
    private void hold() {
        if(currentGamePiece == GamePiece.CONE) {
            setIntakeMotor(-SuperStructureConstants.INTAKE_HOLD_POWER,SuperStructureConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
            
        } else {
            setIntakeMotor(-SuperStructureConstants.INTAKE_HOLD_POWER,SuperStructureConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        }
    }
    private void score() {
        if(currentGamePiece == GamePiece.CUBE) {
            setIntakeMotor(-SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            setIntakeMotor(SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
        }
    }
    private void idle() {
        setIntakeMotor(0, 0);
    }

    /**
     * to be called with a whileTrue trigger binding
     */
    public Command intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {this.currentGamePiece = piece; intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue trigger binding
     */
    public Command scoreGamePiece() {
        return this.startEnd(() ->  {score(); this.currentGamePiece = GamePiece.NONE;},() -> idle());
    }
}
