package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionDualIO {
    
    @AutoLog
    public static class VisionDualInputs {
        public Pose3d rfovBlue = new Pose3d();
        public Pose3d rfovRed = new Pose3d();
        public Pose3d lfovBlue = new Pose3d();
        public Pose3d lfovRed = new Pose3d();
        public double rLatency = 0.0;
        public double lLatency = 0.0;
        public int rNumTags = 0;
        public int lNumTags = 0;
        public boolean rValid = false;
        public boolean lValid = false;
        //constructor if needed for some inputs
        VisionDualInputs() {
        }
    }
    /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionDualInputs inputs) {}
}
