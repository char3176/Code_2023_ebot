package team3176.robot.commands.drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.CommandBase;

import team3176.robot.subsystems.drivetrain.Drivetrain;

public class FeederPID extends CommandBase{
    Drivetrain m_Drivetrain;
    PIDController xController = new PIDController(0.5,0.0,0.0);
    PIDController yController = new PIDController(0.1,0.0,0.0);;
    PIDController wController = new PIDController(1.0,0.0,0.0);;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    NetworkTable vision;
    InterpolatingTreeMap<Double,Double> blueRight = new InterpolatingTreeMap<Double,Double>();
    InterpolatingTreeMap<Double,Double> blueLeft = new InterpolatingTreeMap<Double,Double>();
    public FeederPID() {
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
        vision = NetworkTableInstance.getDefault().getTable("limelight");
        blueRight.put(1.8,-20.0);
        blueRight.put(0.33,-5.9);
        blueRight.put(0.11,-0.8);
        blueRight.put(0.0,0.0);
        blueLeft.put(1.6,28.0);
        blueLeft.put(0.46,18.1);
        blueLeft.put(0.19,13.0);
        blueLeft.put(0.08,7.4);
    }
    @Override
    public void initialize(){
        m_Drivetrain.setSpinLock(true);
        m_Drivetrain.setSpinLockAngle();
    }
    @Override
    public void execute() {
        ta = vision.getEntry("ta").getDouble(0.0);
        ty = vision.getEntry("ty").getDouble(0.0);
        tx = vision.getEntry("tx").getDouble(0.0);
        
        m_Drivetrain.drive (MathUtil.clamp(xController.calculate(ta, 1.5),-2.0,2.0),
                            (MathUtil.clamp(yController.calculate(tx,blueLeft.get(ta)),-2.0,2.0)),
                            0.0);
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
