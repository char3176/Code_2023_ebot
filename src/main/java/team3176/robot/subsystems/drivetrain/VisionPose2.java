package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPose2 {
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
    public boolean ltv;
    public boolean rtv;
    private int lnumtargets = 0;
    private int rnumtargets = 0;

    public VisionPose2(){
        lResults = LimelightHelpers.getLatestResults(llLeftString).targetingResults;
        rResults = LimelightHelpers.getLatestResults(llRightString).targetingResults;
        poseEstimateBlue = new Pose2d();
        poseEstimateRed = new Pose2d();
    }
    
    public void updateLLlfovNetworkTables() { //update target coords.
        if (lResults.valid) {
            ltv = LimelightHelpers.getTV(llLeftString);
            if (ltv){
                ltarget3D = LimelightHelpers.getBotPose_TargetSpace(llLeftString);
                lblue3D = LimelightHelpers.getBotPose_wpiBlue(llLeftString);
                lred3D = LimelightHelpers.getBotPose_wpiRed(llLeftString);
                ltid = LimelightHelpers.getFiducialID(llLeftString);
                lnumtargets = rResults.targets_Fiducials.length;
            }
        }
    }

    public void updateLLrfovNetworkTables() { //update target coords.
        if (rResults.valid) {
            rtv = LimelightHelpers.getTV(llRightString);
            if(rtv) {
                rtarget3D = LimelightHelpers.getBotPose_TargetSpace(llRightString);
                rblue3D = LimelightHelpers.getBotPose_wpiBlue(llRightString);
                rred3D = LimelightHelpers.getBotPose_wpiRed(llRightString);
                rtid = LimelightHelpers.getFiducialID(llRightString);
                rnumtargets = rResults.targets_Fiducials.length;
            }
        }
    }

    public boolean isValid() {
        return ltv || rtv ;
    }
    public void updatePose() {
        if(ltv && rtv ) {
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
        updateLLlfovNetworkTables();
        updateLLrfovNetworkTables();
        updatePose();
        SmartDashboard.putNumber("visionBlueX", poseEstimateBlue.getX());
        SmartDashboard.putNumber("visionBlueY", poseEstimateBlue.getY());
        SmartDashboard.putNumber("visionRightX", rblue3D[0]);
        SmartDashboard.putNumber("visionRightY", rblue3D[1]);
        SmartDashboard.putNumber("visionLeftX", lblue3D[0]);
        SmartDashboard.putNumber("visionLeftY", lblue3D[1]);
    }
}
