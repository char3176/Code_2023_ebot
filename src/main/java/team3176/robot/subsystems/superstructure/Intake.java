package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    public enum intakeStates {IDLE,HOLD,INTAKE,SCORE};
    public GamePiece currentGamePiece = GamePiece.NONE;
    private intakeStates intakeState = intakeStates.IDLE;
    private Intake() {
        intake = new CANSparkMax(Hardwaremap.intake_CID, MotorType.kBrushless);
    }
    public void setIntakeMotor(double percent, int amps) {
        intake.set(percent);
        intake.setSmartCurrentLimit(amps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
    }

    public void setState(intakeStates state) {
        this.intakeState = state;
    }
    /**
     * to be called with a whileTrue trigger binding
     */
    public Command intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {setState(intakeStates.INTAKE); this.currentGamePiece = piece;},() -> setState(intakeStates.HOLD));
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue trigger binding
     */
    public Command score() {
        return this.startEnd(() ->  {setState(intakeStates.INTAKE); this.currentGamePiece = GamePiece.NONE;},() -> setState(intakeStates.IDLE));
    }
    
    @Override
    public void periodic() {
        
        switch(intakeState) {
            case HOLD:
                if(currentGamePiece == GamePiece.CONE) {
                    setIntakeMotor(-SuperStructureConstants.INTAKE_HOLD_POWER,SuperStructureConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
                    
                } else {
                    setIntakeMotor(-SuperStructureConstants.INTAKE_HOLD_POWER,SuperStructureConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
                }
                break;
            case IDLE:
                setIntakeMotor(0, 0);
                break;
            case INTAKE:
                if(currentGamePiece == GamePiece.CUBE) {
                   setIntakeMotor(SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
                }
                else if(currentGamePiece == GamePiece.CONE) {
                    setIntakeMotor(-SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
                }
                break;
            case SCORE:
                if(currentGamePiece == GamePiece.CUBE) {
                    setIntakeMotor(-SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
                }
                else if(currentGamePiece == GamePiece.CONE) {
                    setIntakeMotor(SuperStructureConstants.INTAKE_OUTPUT_POWER,SuperStructureConstants.INTAKE_CURRENT_LIMIT_A);
                }
                break;
        }
    }
}
