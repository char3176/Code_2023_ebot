package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.CommandBase;


import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;

public class FeederPID extends CommandBase{
    Drivetrain m_Drivetrain;
    PIDController xController = new PIDController(2.0,0.0,0.0);
    PIDController yController = new PIDController(0.06,0.0,0.0);;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    NetworkTable vision;
    InterpolatingTreeMap<Double,Double> offsetTreeR = new InterpolatingTreeMap<Double,Double>();
    InterpolatingTreeMap<Double,Double> offsetTreeL = new InterpolatingTreeMap<Double,Double>();
    InterpolatingTreeMap<Double,Double> offsetTree;
    Alliance alliance;
    String side;
    public FeederPID(String side) {
        this.side = side;
        alliance = DriverStation.getAlliance();
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
        vision = NetworkTableInstance.getDefault().getTable("limelight");
        
        offsetTreeR.put(1.8,-20.0);
        offsetTreeR.put(0.33,-5.9);
        offsetTreeR.put(0.11,-0.8);
        offsetTreeR.put(0.0,0.0);

        offsetTreeL.put(1.6,22.0);
        offsetTreeL.put(0.46,7.1);
        offsetTreeL.put(0.19,2.0);
        offsetTreeL.put(0.0,0.0);
        
        if(side == "right") {
            offsetTree = offsetTreeR;
        } else {
            offsetTree = offsetTreeL;
        }
    }
    @Override
    public void initialize(){
        m_Drivetrain.setSpinLock(true);
        m_Drivetrain.setSpinLockAngle(180.0);
        xController.setP(2.0); 
    }
    @Override
    public void execute() {
        ta = vision.getEntry("ta").getDouble(0.0);
        ty = vision.getEntry("ty").getDouble(0.0);
        tx = vision.getEntry("tx").getDouble(0.0);
        double tv = vision.getEntry("tv").getDouble(0.0);
        double txSetpoint = 0.0;
        if (ta > 1.1 );{
            if(side == "right") {
                txSetpoint = -20;
            } else {
                txSetpoint = 20;
            }
        }
        if (Math.abs(m_Drivetrain.getPoseYawWrapped().getDegrees()) > 170 && tv != 0.0) {
            m_Drivetrain.drive (MathUtil.clamp(xController.calculate(ta, 1.5),-1.5,1.5),
                            (MathUtil.clamp(yController.calculate(tx,txSetpoint),-1.5,1.5)),
                            0.0, coordType.ROBOT_CENTRIC);
        } else m_Drivetrain.drive (Math.pow(10,-7),Math.pow(10,-7),Math.pow(10,-7));
        SmartDashboard.putNumber("yawWrapped", m_Drivetrain.getPoseYawWrapped().getDegrees());
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
