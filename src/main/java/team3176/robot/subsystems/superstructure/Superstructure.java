package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.superstructure.arm.armAnalogDown;
import team3176.robot.commands.superstructure.claw.ClawInhaleCone;
import team3176.robot.commands.superstructure.claw.ClawInhaleCube;
import team3176.robot.commands.superstructure.intakecone.IntakeConeExtendSpin;
import team3176.robot.commands.superstructure.intakecone.IntakeConeRetractSpinot;
import team3176.robot.commands.superstructure.intakecube.*;
import team3176.robot.subsystems.superstructure.IntakeCone;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private Arm m_Arm;
    private Claw m_Claw;
    private IntakeCube m_IntakeCube;
    private IntakeCone m_IntakeCone;
    public Superstructure() {
        m_Arm = Arm.getInstance();
        m_Claw = Claw.getInstance();
        m_IntakeCube = IntakeCube.getInstance();
        m_IntakeCone = IntakeCone.getInstance();
    }
    public static Superstructure getInstance() {
        if (instance == null){instance = new Superstructure();}
        return instance;
    }
    
    

    public static enum GamePiece {CUBE, CONE, NONE};
    

    public Command groundCube() {
        return new IntakeGroundCube().andThen(this.prepareCarry());
    }

    public Command groundCone()
    {
        return new ParallelCommandGroup(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CATCH_POS), 
                                        new IntakeConeExtendSpin(),
                                        new ClawInhaleCone())
                    .until(() -> this.m_Claw.getLinebreakThree() == false)
                    .andThen(m_IntakeCone.coneToClaw())
                    .andThen(new IntakeConeRetractSpinot())
                    .andThen(this.prepareCarry());
    }

    public Command clawIntakeCube()
    {
        return new InstantCommand(() -> m_Claw.intakeGamePiece(GamePiece.CUBE));
    }

    /* 
    public Command poopCube() {
        return new ParallelCommandGroup(new IntakeExtendFreeSpin())
                    .andThen(this.preparePoop())
                    .andThen(m_Claw.scoreGamePiece())
                    .andThen(this.prepareCarry())
                    .andThen(new IntakeRetractSpinot());
            //May need to add Wait Cmds in the above logic
    }
    */
    public Command scoreGamePieceAuto() {
        return m_Claw.determineGamePiece()
                .andThen(m_Arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(3.0)
                    .andThen(m_Claw.scoreGamePiece())
                    .andThen(this.prepareCarry()));
    }
    public Command scoreFirstGamePieceAuto() {
        return m_Claw.determineGamePiece()
                .andThen(m_IntakeCone.extendAndFreeSpin().withTimeout(1.0)
                .alongWith(m_Arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(3.0)
                .andThen(new WaitCommand(0.5))
                    .andThen(m_Claw.scoreGamePiece().withTimeout(1.0))
                    .andThen(this.prepareCarry())));
    }
    public Command scoreGamePieceHigh()
    {
        return m_Claw.determineGamePiece()
                .andThen(m_Arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(3.0))
                .andThen(new WaitCommand(0.5))
                .andThen(m_Claw.scoreGamePiece().withTimeout(1.0))
                .andThen(this.prepareCarry());
    }
    public Command scoreCubeLow() {
        return m_Arm.armSetPosition(SuperStructureConstants.ARM_ZERO_POS)
        .andThen(new WaitCommand(0.5))
        .andThen(m_Claw.scoreGamePiece())
        .andThen(this.prepareCarry());
    }
    public Command scoreGamePieceLowAuto()
    {
        return m_Claw.determineGamePiece()
                .andThen(m_Arm.armSetPositionBlocking(SuperStructureConstants.ARM_CATCH_POS).withTimeout(3.0))
                .andThen(new WaitCommand(0.5))
                .andThen(m_Claw.scoreGamePiece().withTimeout(1.0))
                .andThen(this.prepareCarry());
    }
    public Command intakeCubeHumanPlayer() {
        return new ParallelCommandGroup(new ClawInhaleCube(), m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS));
    }

    public Command intakeConeHumanPlayer() {
        return new ParallelCommandGroup(new ClawInhaleCone(), m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS));
    }
    
    public Command preparePoop() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_ZERO_POS);
    }
    public Command prepareCarry() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS);
    }
    public Command prepareCatch() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CATCH_POS);
    }
    public Command prepareScoreMid() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_MID_POS);
    }
    public Command prepareScoreHigh() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS);
    }
    
}
