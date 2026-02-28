// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.wpilibj.xrp.XRPGyro;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
  
     private XRPMotor leftMotor = new XRPMotor(0);
     private XRPMotor rightMotor = new XRPMotor(1);

     private DifferentialDrive dDrive = new DifferentialDrive(leftMotor, rightMotor);
  
     private XboxController joy = new XboxController(0);

     // The XRP has onboard encoders that are hardcoded
     // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);

    private final XRPRangefinder rangeFinder = new XRPRangefinder();
    private final XRPReflectanceSensor reflectSensor = new XRPReflectanceSensor();

    private final XRPGyro gyro = new XRPGyro();

    //from documentation/frc-docs/docs/xrp-robot/getting-to-know-xrp.html
    //wheel diameter = 60mm (2.3622”)
    //encoder tick count per revolution = 585
    private final double kDriveTick2Inch = Math.PI * 2.3622/585;

    //speed constant
    final double kP = 0.04;

    private int setpoint = 0;
    private int lastpoint = 0;

    private double leftsensorPosition = 0;
    private double rightsensorPosition = 0;
    private double averagesensorPosition = 0;

    private double lefterror = 0;
    private double righterror = 0;
    private double averageerror = 0;

    private double leftoutputSpeed = 0;
    private double rightoutputSpeed = 0;
    private double averageoutputSpeed = 0;

    //setting starting value to max; 4000mm or 157.4803 inches
    private double currentRange = 157.4803;

    //values for the line reflectance sensor
    private double leftReflectVal = 0;
    private double rightReflectVal = 0;
    private double reflectErr = 0;
    private double rightReflectMod = 0;
    private double leftReflectMod = 0;

    private boolean followLine = false;

  public Robot() {
    rightMotor.setInverted(true);
  }

  @Override
  public void autonomousInit() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    gyro.reset();  
  }
  
  @Override
  public void autonomousPeriodic() {
    if (joy.getXButtonPressed()){
      followLine = !followLine;
    }
    //head to goal if a button pressed
    //head back to start if b button pressed
    if (joy.getAButtonPressed()){
        setpoint = 36;
    } else if (joy.getBButtonPressed()){
        setpoint = 0;   
    }

    //check for possible collision and stop if too close
    currentRange = rangeFinder.getDistanceInches();
    if(currentRange <= 4 && setpoint != 0){
      setpoint = lastpoint;
    }

    //if following a line get the current values
    rightReflectVal = reflectSensor.getRightReflectanceValue();
    leftReflectVal = reflectSensor.getLeftReflectanceValue();
    //since right is first positive means turn right, negative turn left
    reflectErr = rightReflectVal - leftReflectVal;

    //todo scale values to [-1,1]
    leftsensorPosition = m_leftEncoder.get() * kDriveTick2Inch;
    rightsensorPosition = m_rightEncoder.get() * kDriveTick2Inch;
    averagesensorPosition = (leftsensorPosition + rightsensorPosition)/2; 

    //record where the xrp was prior to updating output speed
    lastpoint = (int)(averagesensorPosition);

    lefterror = setpoint - leftsensorPosition;
    righterror = setpoint - rightsensorPosition;
    averageerror = (lefterror + righterror)/2;

    leftoutputSpeed = kP * lefterror;
    rightoutputSpeed = kP  * righterror;
    
    if(followLine){
      //pos = turn right; neg = turn left;
      if(reflectErr > 0){
        rightReflectMod = -.05;
        leftReflectMod = .1;
      }else if(reflectErr < 0){
        leftReflectMod = -.05;
        rightReflectMod = 1;
      }else{
        leftReflectMod = 0;
        rightReflectMod = 0;
      }
      leftoutputSpeed += leftReflectMod;
      rightoutputSpeed += rightReflectMod;
    }

    averageoutputSpeed = (leftoutputSpeed + rightoutputSpeed)/2;

    leftMotor.set(leftoutputSpeed);
    rightMotor.set(rightoutputSpeed);
    
  }

  @Override
  public void robotPeriodic(){
    SmartDashboard.putBoolean("followingLine", followLine);
    SmartDashboard.putNumber("currentSetPoint", setpoint);
    SmartDashboard.putNumber("LastPoint", lastpoint);
    SmartDashboard.putNumber("rangeFindDist", currentRange);

    SmartDashboard.putNumber("leftEncoder value", m_leftEncoder.get());
    SmartDashboard.putNumber("rightEncoder value", m_rightEncoder.get());

    SmartDashboard.putNumber("leftsensorPosition value", leftsensorPosition);
    SmartDashboard.putNumber("rightsensorPosition value", rightsensorPosition);
    SmartDashboard.putNumber("averagesensorPosition", averagesensorPosition);
    SmartDashboard.putNumber("lefterror value", lefterror);
    SmartDashboard.putNumber("righterror value", righterror);
    SmartDashboard.putNumber("averageerror", averageerror);

    SmartDashboard.putNumber("leftoutputSpeed value", leftoutputSpeed);
    SmartDashboard.putNumber("rightoutputSpeed value", rightoutputSpeed);
    SmartDashboard.putNumber("outputSpeed", averageoutputSpeed);

    SmartDashboard.putNumber("reflectSensor_LEFT", leftReflectVal);
    SmartDashboard.putNumber("reflectSensor_RIGHT", rightReflectVal);
   
   
  }
  @Override
  public void teleopInit() {
    m_leftEncoder.reset();
    m_rightEncoder.reset(); 

  }

  @Override
  public void teleopPeriodic() {

    dDrive.arcadeDrive(-joy.getLeftY(),-joy.getRightX());

  }
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
