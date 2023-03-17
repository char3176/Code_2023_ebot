package team3176.robot.commands.drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import team3176.robot.subsystems.drivetrain.Drivetrain;

public class FeederPID extends CommandBase{
    Drivetrain m_Drivetrain;
    PIDController xController = new PIDController(1.0,0.0,0.0);
    PIDController yController = new PIDController(1.0,0.0,0.0);;
    PIDController wController = new PIDController(1.0,0.0,0.0);;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    NetworkTable vision;
    public FeederPID() {
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
        vision = NetworkTableInstance.getDefault().getTable("limelight");
    }
    @Override
    public void initialize(){
    }
    @Override
    public void execute() {
        ta =-1 * vision.getEntry("ta").getDouble(0.0);
        ty =-1 * vision.getEntry("ty").getDouble(0.0);
        tx =-1 * vision.getEntry("tx").getDouble(0.0);
        
        m_Drivetrain.drive(xController.calculate(ta,30),
                            yController.calculate(tx,0),
                            0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
    }
}
