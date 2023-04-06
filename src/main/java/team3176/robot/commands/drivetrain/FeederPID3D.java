package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;


import team3176.robot.subsystems.drivetrain.Drivetrain;

public class FeederPID3D extends CommandBase{
    Drivetrain m_Drivetrain;
    PIDController xController = new PIDController(2.0,0.0,0.0);
    PIDController yController = new PIDController(2.0,0.0,0.0);
    Pose2d RedRight = new Pose2d(1.17, 7.44, Rotation2d.fromDegrees(180));
    Pose2d RedLeft = new Pose2d(1.17, 6.17, Rotation2d.fromDegrees(180));
    Pose2d BlueRight = new Pose2d(15.44, 6.17, Rotation2d.fromDegrees(0.0));
    Pose2d BlueLeft = new Pose2d(15.44, 7.44, Rotation2d.fromDegrees(0.0));
    Pose2d targetPose;
    Alliance alliance;
    String side;
    public FeederPID3D(String side) {
        this.side = side;
        alliance = DriverStation.getAlliance();
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
        if(side == "right") {
            if(DriverStation.getAlliance() == Alliance.Red) {
                targetPose = RedRight;
            } else {
                targetPose = BlueRight;
            }
        } else {
            if(DriverStation.getAlliance() == Alliance.Red) {
                targetPose = RedLeft;
            } else {
                targetPose = BlueLeft;
            }
        }
    }
    @Override
    public void initialize(){
        m_Drivetrain.setSpinLock(true);
        m_Drivetrain.setSpinLockAngle(targetPose.getRotation().getDegrees()); 
    }
    @Override
    public void execute() {
        Pose2d cam_pose = m_Drivetrain.getVisionPoseBlue();
        double reverseAxis = DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0;
        if (m_Drivetrain.isVisionValid()) {
            m_Drivetrain.drive(MathUtil.clamp(reverseAxis*xController.calculate(cam_pose.getX(), targetPose.getX()),-1.5,1.5),
                            (MathUtil.clamp(reverseAxis*yController.calculate(cam_pose.getY(),targetPose.getY()),-1.5,1.5)),
                            0.0);
        } else m_Drivetrain.drive (Math.pow(10,-7),Math.pow(10,-7),Math.pow(10,-7));
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        m_Drivetrain.setSpinLock(false);
    }
}
