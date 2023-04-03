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
    PIDController yController = new PIDController(1.0,0.0,0.0);;
    double ltx, rtx, lty, rty, lta, rta, tx, ty, ta, tv = 0;
    int numLimelights = 2;
    double deadband;
    NetworkTable vision, limelight_lfov, limelight_rfov;
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
        //vision = NetworkTableInstance.getDefault().getTable("limelight");
        limelight_lfov = NetworkTableInstance.getDefault().getTable("limelight-lfov");
        limelight_rfov = NetworkTableInstance.getDefault().getTable("limelight-rfov");
        
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
        m_Drivetrain.setSpinLockAngle(0.0);
        xController.setP(2.0); 
        deadband = 1;
    }
    @Override
    public void execute() {
        lta = limelight_lfov.getEntry("ta").getDouble(0.0);
        rta = limelight_rfov.getEntry("ta").getDouble(0.0);
        ta = (lta + rta) / numLimelights;
        ltx = limelight_lfov.getEntry("tx").getDouble(0.0);
        rtx = limelight_rfov.getEntry("tx").getDouble(0.0);
        tx = (ltx + rtx) / numLimelights;
        lty = limelight_lfov.getEntry("ty").getDouble(0.0);
        rty = limelight_rfov.getEntry("ty").getDouble(0.0);
        ty = (lty + rty) / numLimelights;
        double ltv = limelight_lfov.getEntry("tv").getDouble(0.0);
        double rtv = limelight_rfov.getEntry("tv").getDouble(0.0);
        tv = (ltv == 1 || rtv == 1)  ? 1 : 0;
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("ta", ta);
        SmartDashboard.putNumber("tv", tv);
        double txSetpoint = 0.0;
        if (ta > 1.1 );{
            if(side == "right") {
                txSetpoint = 0 ; //-20;
            } else {
                txSetpoint = 0 ; //20;
            }
        }
        //if (Math.abs(m_Drivetrain.getPoseYawWrapped().getDegrees()) > 170 && tv != 0.0) {
        if (tv!= 0.0) {
        //    m_Drivetrain.drive (MathUtil.clamp(xController.calculate(ta, 1.5),-1.5,1.5),
        //                    (MathUtil.clamp(yController.calculate(tx,txSetpoint),-1.5,1.5)),
        //                    0.0, coordType.ROBOT_CENTRIC);
            if ((tx < (txSetpoint-deadband) || (tx > (txSetpoint+deadband)))) {
                m_Drivetrain.drive(0,
                    (MathUtil.clamp(-1 * yController.calculate(tx,txSetpoint), -1.5, 1.5)),
                    0);
            }
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
