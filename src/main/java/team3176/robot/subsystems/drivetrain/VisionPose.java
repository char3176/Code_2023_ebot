package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPose {
    private LimelightHelpers.Results lResults;
    private LimelightHelpers.Results rResults;
    private static String llLeftString = "limelight-lpov";
    private static String llRightString = "limelight-rpov";
    private NetworkTable llLeft = NetworkTableInstance.getDefault().getTable(llLeftString);
    private NetworkTable llRight = NetworkTableInstance.getDefault().getTable(llRightString);
    private Pose2d poseEstimateBlue;
    private Pose2d poseEstimateRed;
    public double[] ltarget3D;
    public double[] rtarget3D;
    public double[] lblue3D;
    public double[] lred3D;
    public double[] rblue3D;
    public double[] rred3D;
    public double ltid;
    public double rtid;
    public double ltv;
    public double rtv;
    private int lnumtargets = 0;
    private int rnumtargets = 0;

    public VisionPose(){
        lResults = LimelightHelpers.getLatestResults(llLeftString).targetingResults;
        rResults = LimelightHelpers.getLatestResults(llRightString).targetingResults;
        poseEstimateBlue = new Pose2d();
        poseEstimateRed = new Pose2d();
    }
    
    public void updateLLNetworkTables() { //update target coords.
        ltv = llLeft.getEntry("tv").getDouble(0);
        rtv = llRight.getEntry("tv").getDouble(0);
        if(ltv > 0.0) {
            ltarget3D = llLeft.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            lblue3D = llLeft.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
            lred3D = llLeft.getEntry("botpose_wpired").getDoubleArray(new double[7]);
            ltid = llLeft.getEntry("tid").getDouble(0);
            lnumtargets = lResults.targets_Fiducials.length;
        }
        if (rtv > 0.0){
            rtarget3D = llRight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            rblue3D = llRight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
            rred3D = llRight.getEntry("botpose_wpired").getDoubleArray(new double[7]);
            rtid = llRight.getEntry("tid").getDouble(0);
            rnumtargets = rResults.targets_Fiducials.length;
        }
    }
    public boolean isValid() {
        return ltv != 0 || rtv != 0;
    }
    public void updatePose() {
        if(ltv == 0 && rtv == 0) {
            //reset pose estimate to 0 indicating no seen tags
            poseEstimateBlue = new Pose2d();
            poseEstimateRed = new Pose2d();
        }
        else if(lnumtargets > rnumtargets) {
            poseEstimateBlue = LimelightHelpers.toPose2D(lblue3D);
            poseEstimateRed = LimelightHelpers.toPose2D(lred3D);
        } else if(lnumtargets < rnumtargets) {
            poseEstimateBlue = LimelightHelpers.toPose2D(rblue3D);
            poseEstimateRed = LimelightHelpers.toPose2D(rred3D);
        } else {
            //equal number of targets we will average
            double blueX = (LimelightHelpers.toPose2D(rblue3D).getX() + LimelightHelpers.toPose2D(lblue3D).getX())/2.0;
            double blueY = (LimelightHelpers.toPose2D(rblue3D).getY() + LimelightHelpers.toPose2D(lblue3D).getY())/2.0;
            poseEstimateBlue = new Pose2d(blueX, blueY, LimelightHelpers.toPose2D(rblue3D).getRotation());
            double redX = (LimelightHelpers.toPose2D(rred3D).getX() + LimelightHelpers.toPose2D(lred3D).getX())/2.0;
            double redY = (LimelightHelpers.toPose2D(rred3D).getY() + LimelightHelpers.toPose2D(lred3D).getY())/2.0;
            poseEstimateRed = new Pose2d(redX, redY, LimelightHelpers.toPose2D(rred3D).getRotation());
        }
    }
    public Pose2d getPoseRed() {
        return poseEstimateRed;
    }
    public Pose2d getPoseBlue() {
        return poseEstimateBlue;
    }
    public void periodic() {
        lResults = LimelightHelpers.getLatestResults(llLeftString).targetingResults;
        rResults = LimelightHelpers.getLatestResults(llRightString).targetingResults;
        updateLLNetworkTables();
        updatePose();
        SmartDashboard.putNumber("visionBlueX", poseEstimateBlue.getX());
        SmartDashboard.putNumber("visionBlueY", poseEstimateBlue.getY());
        SmartDashboard.putNumber("visionRightX", rblue3D[0]);
        SmartDashboard.putNumber("visionRightY", rblue3D[1]);
        SmartDashboard.putNumber("visionLeftX", lblue3D[0]);
        SmartDashboard.putNumber("visionLeftY", lblue3D[1]);
    }
}
