package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

import team3176.robot.subsystems.superstructure.ClawIO;
import team3176.robot.subsystems.superstructure.ClawIO.ClawIOInputs;
import team3176.robot.subsystems.superstructure.ClawIO.ClawIOHardware;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private final ClawIO io;
    private final ClawIOInputs inputs = new ClawIOInputs();
    public final ClawIOHardware hardware = new ClawIOHardware();
    public GamePiece currentGamePiece = GamePiece.CONE;

    private Claw(ClawIO io) {this.io = io;}

    public void setClawMotor(double percent, int amps) {
        hardware.setClawVelocity(percent);
        hardware.setClawAmps(amps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", hardware.getClawAmps());
        SmartDashboard.putNumber("intake motor temperature (C)", hardware.getClawTemp());
    }

    //states now implemented as functions
    
    public void intake() {
        //System.out.println("m_Claw.intake()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    
    public void hold() {
        //System.out.println("m_Claw.hold()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            //idle(); 
        } else {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER * SuperStructureConstants.CLAW_HOLD_CONE_FACTOR,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }

    public void score() {
        //System.out.println("m_Claw.score()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    public void idle() {
        //System.out.println("m_Claw.idle()");
        setClawMotor(0, 0);
    }

    public void setCurrentGamePiece(GamePiece piece) {
        //System.out.println("m_Claw.setCurrentGamePiece() to " + piece);
        this.currentGamePiece = piece;
    }

    public boolean isEmpty() {
        return hardware.getLinebreakOne() || hardware.getLinebreakTwo();
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
        return this.startEnd(() ->  {this.currentGamePiece = piece; intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue or onTrue trigger binding
     */
    public Command scoreGamePiece() {  
        return this.run(() ->  {score(); this.currentGamePiece = GamePiece.NONE;})
                    .until(() -> this.isEmpty())
                    .andThen(new WaitCommand(1.5))
                    .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle());
    }

    //more examples of command composition and why its awesome!!
    public Command intakeCone() {
        return this.intakeGamePiece(GamePiece.CONE).until(() -> hardware.getLinebreakTwo());
    }
    public Command intakeCube() {
        return this.intakeGamePiece(GamePiece.CUBE).until(() -> hardware.getLinebreakOne());
    }
    public Command determineGamePiece() {
        return this.runOnce( () -> {
            if(this.hardware.getLinebreakOne()) {
                this.currentGamePiece = GamePiece.CONE;
                hold();
            } else if(this.hardware.getLinebreakTwo()) {
                this.currentGamePiece = GamePiece.CUBE;
                hold();
            }
        });
    }



    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    Logger.getInstance().recordOutput("Claw/Velocity", getVelocity());
    Logger.getInstance().recordOutput("Claw/LinebreakOne", getIsLinebreakOne());
    Logger.getInstance().recordOutput("Claw/LinebreakTwo", getIsLinebreakTwo());
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("linebreakOne",hardware.getLinebreakOne());
    SmartDashboard.putBoolean("linebreakTwo",hardware.getLinebreakTwo());
    SmartDashboard.putBoolean("linebreakThree", hardware.getLinebreakThree());
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

   public boolean getIsLinebreakThree()
   {
    return inputs.isLinebreakThree;
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
