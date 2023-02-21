package team3176.robot.constants;

public class SuperStructureConstants {
  
    /**
   * How many amps the arm motor can use.
   */
  public static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  public static final double ARM_OUTPUT_POWER = 0.2;

  /**
   * How many amps the claw can use while picking up
   */
  public static final int CLAW_CURRENT_LIMIT_A = 25;
  public static final int CLAW_AMPS = 25;

  /**
   * How many amps the claw can use while holding
   */
  public static final int CLAW_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for inhaling
   */
  public static final double CLAW_OUTPUT_POWER_CUBE = .3;
  public static final double CLAW_OUTPUT_POWER_CONE = 1.0;

  /**
   * Percent output for holding
   */
  public static final double CLAW_HOLD_POWER = 0.14;

  public static final int ARM_ENCODER_OFFSET = 0; 

  public static final int ARM_kP = 0;
  public static final int ARM_kI = 0;
  public static final int ARM_kD = 0;
  public static final double ARM_TOLERANCE = 4;


  public static final double ARM_POOP_POS = 170.947;
  public static final double ARM_CARRY_POS = 209.795;
  public static final double ARM_CATCH_POS = 11.77;
  public static final double ARM_MID_POS = 277.734;
  public static final double ARM_HIGH_POS = 310.869;
}
