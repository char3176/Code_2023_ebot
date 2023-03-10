package team3176.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.vision.Vision;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoubleSubscriber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FeederAlign {
    private Drivetrain m_Drivetrain;
    public NetworkTableInstance tableInstance;
    public NetworkTable limelightTable;
    public DoubleTopic aprilIDTopic;
    public DoubleSubscriber aprilID;
    public Double wantedAprilID;
    public Alliance alliance;

    public FeederAlign(){
        m_Drivetrain = Drivetrain.getInstance();
    
    }
    
    public void initialize(){
        m_Drivetrain.setBrakeMode();
        tableInstance = NetworkTableInstance.getDefault();
        limelightTable = tableInstance.getTable("limelight");
        if (DriverStation.isFMSAttached() && (alliance == null)) {
            alliance = DriverStation.getAlliance();
            if (alliance == Alliance.Red){
                wantedAprilID = 5.0;
            } else if (alliance == Alliance.Blue){
                wantedAprilID = 4.0;
            } else if (alliance == Alliance.Invalid){
                wantedAprilID = 9.0;
            }
         }

    }
    public void execute(){
        Pose2d pose = m_Drivetrain.getPose();
        double xpos = pose.getX();
        double ypos = pose.getY();
        double limelightXpos = 69; //placeholder until we get the bot
        double limelightYpos = 69; //placeholder until we get the bot
        double limelightYaw = 69; //placeholder until we get the bot
        aprilIDTopic = limelightTable.getDoubleTopic("tid");
        aprilID = aprilIDTopic.subscribe(0.0);
        double forward = 0.0;
        if (aprilID.getAsDouble() == wantedAprilID ){
            
            
        }
    }
}

