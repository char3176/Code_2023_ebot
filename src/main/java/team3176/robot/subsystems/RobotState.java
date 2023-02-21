// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.RobotStateIO; 
import team3176.robot.subsystems.RobotStateIO.RobotStateIOInputs; 

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.constants.SignalingConstants;

import team3176.robot.subsystems.superstructure.Claw;



public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private int wantedLEDState;
  private Claw m_Claw;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int i = 0;

  private int m_flashcounter = 0;
  private boolean leftflash = false;
  private boolean rightflash = false;
  private boolean crossflash = false;
  private Color leftflcolor = Color.kBlack;
  private Color rightflcolor = Color.kBlack;
  private Color crossflcolor = Color.kBlack;

  private Alliance alliance; 

  enum e_ClawPositionState {
    OPEN,
    CLOSED,
    IDLE
  }

  enum e_ClawRollersState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Claw, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Claw, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_IntakePositionState {
    EXTENDED,
    RETRACTED
  }


  enum e_IntakeDetectsImHolding {
    CONE,
    CUBE,
    NOTHING,
    ERROR
  }

  enum e_IntakeMotorState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Intake, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Intake, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_ArmShoulderPositionState {
    IsUP,  //Means Elevator is in up postion
    IsDOWN  // Means Elevator is in down position
  }

  enum e_ArmElbowPositionState {
    IsPICKUP,   //Means Arms is in position to receive game element from Intake
    IsHIGHCONE,  // Means Arm is in High Position to deposit cone
    IsHIGHCUBE,  // Means Arm is in High Position to deposit cube
    IsMIDCONE,  // Means Arm is in Mid Position to deposit cone
    IsMIDCUBE,  // Means Arm is in Mid Position to deposit cube
    IsFLOORCONE,  // Means Arm is in Floor High Position to deposit cone 
    IsFLOORCUBE,  // Means Arm is in Floor Position to deposit cube
  }

  public enum e_CurrentGameElementImWanting{
    CONE,
    CUBE,
    NONE
  }

  public enum e_CurrentGameElementImHolding{
    CONE,
    CUBE,
    NONE
  }


  private RobotState(RobotStateIO io) {
    this.io = io;
    m_Claw = Claw.getInstance();
    wantedLEDState = 0;

    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(73);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setSegment(int start, int end, int red, int green, int blue)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setRGB(i, red , green, blue);
      }
    }

    public void setSegment(int start, int end, Color color)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setLED(i, color);
      }
      // m_led.setData(m_ledBuffer);
    }

    public void setleft(Status s)
    {
      leftflcolor = LookUpColor(s);
      leftflash = LookUpFlash(s);
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, leftflcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setright(Status s)
    {
      rightflcolor = LookUpColor(s);
      rightflash = LookUpFlash(s);
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setcrossbar(Status s)
    {
      crossflcolor = LookUpColor(s);
      crossflash = LookUpFlash(s);
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
      m_led.setData(m_ledBuffer);
    }
  private Color LookUpColor(Status s){
    Color c = Color.kBlack;
    switch(s){
      case STABLE: c = Color.kGreen;
      break;
      case OK: c = Color.kDarkRed;
      break;
      case OPTIONALCHECK: c = Color.kLightYellow;
      break;
      case WARNING: c = Color.kFuchsia;
      break;
      case GOOD: c = Color.kLightCoral;
      break;
      case ERROR: c = Color.kLime;
      break;
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case CONEFLASH: c = Color.kOrange;
      break;
      case CUBEFLASH: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }
  private Boolean LookUpFlash(Status s){
      return ((s == Status.CUBEFLASH) || (s == Status.CONEFLASH));
  }

  public void FlashColor()
  {
    m_flashcounter++;
    if (m_flashcounter == 25){
     if (leftflash){
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP,leftflcolor);
     }
     if (rightflash){
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
     }
     if (crossflash){
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
     }
     m_led.setData(m_ledBuffer);
     }
    if (m_flashcounter == 50){
      if (leftflash){
        setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, Color.kBlack);
       }
       if (rightflash){
        setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, Color.kBlack);
       }
       if (crossflash){
        setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, Color.kBlack);
       }
       m_led.setData(m_ledBuffer);
      m_flashcounter = 0;
     }
  }


  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance();
    }
  }

  public void setColorWantState()
  {
    System.out.println("WAS CALLED");
    wantedLEDState = 1;
    if (wantedLEDState == 0)
    {
      setleft(Status.NONE);
      setright(Status.NONE);
      setcrossbar(Status.NONE);
    }
    else if (wantedLEDState == 1)
    {
      setleft(Status.CONEFLASH);
      setright(Status.CONEFLASH);
      setcrossbar(Status.CONEFLASH);
    }
    else if (wantedLEDState == 2)
    {
      setleft(Status.CUBEFLASH);
      setright(Status.CUBEFLASH);
      setcrossbar(Status.CUBEFLASH);
    }

    wantedLEDState++;
    if (wantedLEDState == 3) {wantedLEDState = 0;}
  }

  public Command setColorWantStateCommand()
  {
    return this.runOnce(() -> setColorWantState());
  }

  public static RobotState getInstance() {
    if(instance == null) {instance = new RobotState(new RobotStateIO() {});}
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    FlashColor();

    i++;
    if ((!m_Claw.getLinebreakOne() || !m_Claw.getLinebreakTwo()) && i == 50)
    {
      if (wantedLEDState == 1)
      {
        setleft(Status.CONE);
        setright(Status.CONE);
        setcrossbar(Status.CONE);
      }
      else if (wantedLEDState == 2)
      {
        setleft(Status.CUBE);
        setright(Status.CUBE);
        setcrossbar(Status.CUBE);
      }
      i = 0;
    }
    else if ((m_Claw.getLinebreakOne() || m_Claw.getLinebreakTwo()) && i == 50)
    {
      setleft(Status.NONE);
      wantedLEDState = 0;
      i = 0;
    }
  }

  @Override
  public void simulationPeriodic() {}
}