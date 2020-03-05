/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //The giant, unending list of variables

  WPI_TalonFX frontLeftDrive;
  WPI_TalonFX backLeftDrive;
  WPI_TalonFX frontRightDrive;
  WPI_TalonFX backRightDrive;

  TalonFXConfiguration motorConfig;

  XboxController drivingController;
  XboxController operatingController;

  //used to sqaure input

  double throttleInput;
  double turningInput;

  //used for encoders
  double position;
  double posRot;
  double velocity;
  double velRot;
  double displacement;
  int feetDisplay;
  double inchDisplay;

  //the actual driving objects

  SpeedControllerGroup leftDrive;
  SpeedControllerGroup rightDrive;
  DifferentialDrive driveTrain;

  final int kUnitsPerRevolution = 2048;
  final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
  final double distancePerRot = 8 * Math.PI;

  int printloop = 0;

  //timer used for autonomous driving
  Timer timer;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //motor config set-up
    motorConfig = new TalonFXConfiguration();
    motorConfig.supplyCurrLimit =  new SupplyCurrentLimitConfiguration(true, 60, 70, 1);
    motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    //create left speed drivers
    frontLeftDrive = new WPI_TalonFX(2);
    backLeftDrive = new WPI_TalonFX(3);
    leftDrive = new SpeedControllerGroup(frontLeftDrive, backLeftDrive);
    frontLeftDrive.configAllSettings(motorConfig);
    backLeftDrive.configAllSettings(motorConfig);
    
    
    //create right speed drivers
    frontRightDrive = new WPI_TalonFX(4);
    backRightDrive = new WPI_TalonFX(5);
    rightDrive = new SpeedControllerGroup(frontRightDrive, backRightDrive);
    frontRightDrive.configAllSettings(motorConfig);
    backRightDrive.configAllSettings(motorConfig);

    frontLeftDrive.setSelectedSensorPosition(0);
    backLeftDrive.setSelectedSensorPosition(0);
    frontRightDrive.setSelectedSensorPosition(0);
    backRightDrive.setSelectedSensorPosition(0);
    
    //set ramp rate
    frontLeftDrive.configOpenloopRamp(.2);
    backLeftDrive.configOpenloopRamp(.2);
    frontRightDrive.configOpenloopRamp(.2);
    backRightDrive.configOpenloopRamp(.2);

    //the driving bit
    driveTrain = new DifferentialDrive(leftDrive, rightDrive);
    driveTrain.setSafetyEnabled(false);

    //Controller that controls movement
    drivingController = new XboxController(0);
    //Controller that controls literally everything else
    operatingController = new XboxController(1);

    //numbers used to square movement in teleop
    throttleInput = 0;
    turningInput = 0;

    //global timer set-up, used for autonomous
    timer = new Timer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    position = frontLeftDrive.getSelectedSensorPosition(0); //gets position units from front left motor
    posRot = (double) position/kUnitsPerRevolution/16.48;
    velocity = frontLeftDrive.getSelectedSensorPosition(0);
    velRot = (double) velocity/kUnitsPerRevolution*10/16.48;

    displacement = (distancePerRot*posRot);

    feetDisplay = (int) (displacement/12);
    inchDisplay = (displacement%12);

    printloop++;

    if (printloop%100 == 0){
    System.out.println("Position value: " + position);
    System.out.println("Rotation Position value: " + posRot);
    System.out.println("Velocity value: " + velocity);
    System.out.println("Rotation Velocity value: " + velRot);
    System.out.println("Distance: " + feetDisplay + "ft. " + inchDisplay + "in.");
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *5
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    //start driving at half speed
    timer.start();
    driveTrain.arcadeDrive(.5, 0);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

    if (feetDisplay >= 3){
      driveTrain.arcadeDrive(0, 0);
      timer.stop();
      timer.reset();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //The driving code
    throttleInput = (-drivingController.getY(Hand.kRight));
    turningInput = (drivingController.getX(Hand.kLeft));

    //square the inputs for throttle
    if (throttleInput < 0){
    throttleInput = -(throttleInput*throttleInput);
  }else{
    throttleInput = throttleInput*throttleInput;
  }
  
  //square the inputs for turning
  if (turningInput < 0){
    turningInput = -(turningInput*turningInput);
  }else{
    turningInput = turningInput*turningInput;
  }

  driveTrain.arcadeDrive(throttleInput, turningInput);
}
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
