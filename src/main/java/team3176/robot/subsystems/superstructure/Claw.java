package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.RobotState;
import team3176.robot.subsystems.RobotState.GamePiece;

import team3176.robot.subsystems.superstructure.ClawIO;
import team3176.robot.subsystems.superstructure.ClawIO.ClawIOInputs;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    private RobotState m_RobotState; 
    private CANSparkMax claw;
    private DigitalInput linebreakCube;
    private DigitalInput linebreakCone;
    private DigitalInput linebreakThree;
    private static Claw instance;
    private final ClawIO io;
    private final ClawIOInputs inputs = new ClawIOInputs();
    private Claw(ClawIO io) {
        this.io = io;
        claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
        linebreakCube = new DigitalInput(0);
        linebreakCone = new DigitalInput(2);
        linebreakThree = new DigitalInput(1);
    }
    public void setRobotStateInstance(RobotState st) {
        m_RobotState = st;
    }
    public void setClawMotor(double percent, double amps) {
        claw.set(percent);
        claw.setSmartCurrentLimit(SuperStructureConstants.CLAW_AMPS);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", claw.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", claw.getMotorTemperature());
    }

    //states now implemented as functions
    
    public void intake() {
        //System.out.println("m_Claw.intake()");
        if(m_RobotState.getWantedGamePiece() == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(m_RobotState.getWantedGamePiece() == GamePiece.CONE) {
             setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    
    public void hold() {
        //System.out.println("m_Claw.hold()");
        if(m_RobotState.getRobotState() == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            //idle(); 
        } else {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER * SuperStructureConstants.CLAW_HOLD_CONE_FACTOR,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }

    public void score() {
        //System.out.println("m_Claw.score()");
        if(m_RobotState.getRobotState() == GamePiece.CUBE) {
            setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(m_RobotState.getRobotState() == GamePiece.CONE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    public void idle() {
        //System.out.println("m_Claw.idle()");
        setClawMotor(0, 0);
    }

    

    public boolean getLinebreakCube() //Cube
    {
        return linebreakCube.get();
    }
    
    public boolean getLinebreakCone()  //Cone (Upper)
    {
        return linebreakCone.get();
    }

    public boolean getLinebreakThree()
    {
        return linebreakThree.get();
    }

    public boolean isEmpty() {
        return getLinebreakCube() || getLinebreakCone();
    }

    public static Claw getInstance()
    {
        if (instance == null ) {
            instance = new Claw(new ClawIO() {});
          }
        return instance;
    }

    /**
     * to be called with a whileTrue trigger binding
     */
    public CommandBase intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue or onTrue trigger binding
     */
    public Command scoreGamePiece() {  
        return this.run(() ->  {score(); })
                    .until(() -> this.isEmpty())
                    .andThen(new WaitCommand(0.5))
                    .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle());
    }

    //more examples of command composition and why its awesome!!
    public Command intakeCone() {
        return this.intakeGamePiece(GamePiece.CONE).until(this::getLinebreakCone);
    }
    public Command intakeCube() {
        return this.intakeGamePiece(GamePiece.CUBE).until(this::getLinebreakCube);
    }
    public Command determineGamePiece() {
        return this.runOnce( () -> {
            if(this.linebreakCube.get()) {
                m_RobotState.setCurrentGamePiece(GamePiece.CUBE);
                hold();
            } else if(this.linebreakCone.get()) {
                m_RobotState.setCurrentGamePiece(GamePiece.CONE);
                hold();
            } else if (this.linebreakCube.get() && this.linebreakCone.get()) {
                m_RobotState.setCurrentGamePiece(GamePiece.NONE);
            }
        });
    }



    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    Logger.getInstance().recordOutput("Claw/Velocity", getVelocity());
    Logger.getInstance().recordOutput("Claw/LinebreakCube", getIsLinebreakOne());
    Logger.getInstance().recordOutput("Claw/LinebreakCone", getIsLinebreakTwo());
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("linebreakCube",linebreakCube.get());
    SmartDashboard.putBoolean("linebreakCone",linebreakCone.get());
    SmartDashboard.putBoolean("linebreakThree", linebreakThree.get());
    determineGamePiece();
    /* 
    if (!linebreakOne.get()) {
        setCurrentGamePiece(GamePiece.CUBE);
    } else if (!linebreakTwo.get()) {
        setCurrentGamePiece(GamePiece.CONE);
    }
    if (linebreakOne.get() && linebreakTwo.get()) {
        setCurrentGamePiece(GamePiece.NONE); 
    }
    */
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public boolean getIsLinebreakOne()
   {
    return inputs.isLinebreakOne;
   }

   public boolean getIsLinebreakTwo()
   {
    return inputs.isLinebreakTwo;
   }

   public void runVoltage(double volts)
   {
    io.setVoltage(volts);
   }

   public void setVelocity(double velocity)
   {
    io.setVelocity(velocity);
   }
}
