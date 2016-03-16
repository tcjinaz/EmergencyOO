package org.usfirst.frc.team3853.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {

  //////////////////////////////////////////////////////////////////////////////////////  HARDWARE SUBSYTEMS
  static boolean ControlTypeIsArcade = true;  // true for arcade, false for tank
  static boolean squaredInputs = true;        // desensitize the controls
  
  DriveBase dt;   // object that handles basic drive operations

  Arm arm;        // object that handles the arm

  Ball ball;      // object that handles the ball

  PdpPipe pdp;    // power distribution panel

  Joystick       leftStick;                                        // set to ID 0 in DriverStation
  Joystick       rightStick;                                       // set to ID 1 in DriverStation
  JoystickButton armGo, armDir, ballIn, ballOutPuke, ballOutBarf;

  
  //////////////////////////////////////////////////////////////////////////////////////  MASTER CONTROL VARIABLES
  // counter used to slow down SmartDashboard updates
  int ocCount;
  boolean noLoopDelayProblem;
  String errorStatus;
  boolean sendUpdate;

  // last time operator control was called
  double lastTime;
  double loopCycleTime;
  double timeDiff;

  //////////////////////////////////////////////////////////////////////////////////////  HARDWARE MAPS

  // PWM Map
  static int PWM_LeftDriveMotor  =  0 ;
  static int PWM_RightDriveMotor =  1 ;
  static int PWM_ArmMotor        =  8 ;
  static int PWM_BallMotor       =  9 ;

  // DIO Map
  static int DIO_ArmEncoderA   = 0 ;
  static int DIO_ArmEncoderB   = 1 ;
  static int DIO_ArmLimit      = 2 ;
  static int DIO_BallLimit     = 7 ;

  
  //////////////////////////////////////////////////////////////////////////////////////  Robot ctor
  // Robot constructor
  public Robot() {

    // moving parts
    arm   = new Arm       ( PWM_ArmMotor, DIO_ArmEncoderA, DIO_ArmEncoderB, DIO_ArmLimit );
    ball  = new Ball      ( PWM_BallMotor, DIO_BallLimit );

    // drivetrain control
    dt    = new DriveBase ( PWM_LeftDriveMotor, PWM_RightDriveMotor );
    if ( ControlTypeIsArcade ) {
      // arcade
      leftStick   = new Joystick( 0 );
    } else {
      // tank drive
      leftStick   = new Joystick( 0 );
      rightStick  = new Joystick( 1 );
    }

    // OI joystick buttons
    armGo         = new JoystickButton( leftStick, 1 );
    armDir        = new JoystickButton( leftStick, 2 );
    if ( ControlTypeIsArcade ) {
      ballIn      = new JoystickButton( leftStick, 4 );
      ballOutBarf = new JoystickButton( leftStick, 5 );
      ballOutPuke = new JoystickButton( leftStick, 6 );
    } else {
      ballIn      = new JoystickButton( rightStick, 1 );
      ballOutBarf = new JoystickButton( rightStick, 2 );
      ballOutPuke = new JoystickButton( rightStick, 3 );
    }

    // output control counter; slow down SmartDashboard updates
    ocCount = 0;
    noLoopDelayProblem = true;
    errorStatus = "";
    sendUpdate = true;

    // set up to watch the power distribution panel
    pdp = new PdpPipe();

    // 25mS control loop timimg
    loopCycleTime = 0.025;
    lastTime = Timer.getFPGATimestamp();

  } // end Robot ctor

  
  //////////////////////////////////////////////////////////////////////////////////////
  // robotInit
  public void robotInit() {
    SmartDashboard.putString( "RobotID", "EmergencySAMPLEOO-160314a" );
    errorStatus = "";
    noLoopDelayProblem = true;
    sendUpdate = true;
  }

  
  //////////////////////////////////////////////////////////////////////////////////////
  // operatorContol
  public void operatorControl() {

    ocCount = 0;
    lastTime = Timer.getFPGATimestamp();

    // myRobot.setSafetyEnabled( true );

    while (isOperatorControl() && isEnabled()) {

      if ( Math.floorMod( ocCount, 4 ) == 0 ) {
        // post interesting data back to the dashboard, but not too often
        updateDashboard();
      }
      ocCount += 1;

      ///////////////////////////////////////////////////////////////////////////////// DRIVETRAIN
      if ( Robot.ControlTypeIsArcade ) {
        dt.go( leftStick, squaredInputs );        
      } else {
        dt.go( leftStick, rightStick, squaredInputs );
      }

      ///////////////////////////////////////////////////////////////////////////////// ARM MOTOR
      // arm motor control
      arm.go( armGo.get(), armDir.get() );

      ///////////////////////////////////////////////////////////////////////////////// BALL MOTRO
      // ball motor control
      ball.go( ballIn.get(), ballOutPuke.get(), ballOutBarf.get() );

      ///////////////////////////////////////////////////////////////////////////////// TIMING LOOP CONTROL
      // time control
      // Timer.delay(0.005); // wait for a motor update time
      lookForTimeTrouble();

    } // end while isOperatorControl

  } // end operatorControl

  
  //////////////////////////////////////////////////////////////////////////////////////
  // Operator communications interface
  
  // send data back to the operator
  public void updateDashboard() {

    // don't overdo writing to dashboard
    ocCount += 1;
    if ( Math.floorMod( ocCount, 10 ) == 0 ) {
      arm.sendAll();
      ball.sendAll();
      pdp.sendAll();
    }
  } // end updateDashboard

  
  //////////////////////////////////////////////////////////////////////////////////////
  // Look for time trouble
  public boolean lookForTimeTrouble() {

    timeDiff = Timer.getFPGATimestamp() - (lastTime + 0.025);
    if ( timeDiff < 0.0 ) {
      // this is bad
      errorStatus = "LOOP DELAY PROBLEM";
      if ( noLoopDelayProblem ) {
        sendUpdate = true;
      }
      noLoopDelayProblem = false;

    } else {
      // this is good
      errorStatus = "";
      if ( !noLoopDelayProblem ) {
        sendUpdate = true;
      }
      noLoopDelayProblem = true;
      Timer.delay( timeDiff );
    }
    if ( sendUpdate ) {
      SmartDashboard.putString( "ERROR STATUS", errorStatus );
      SmartDashboard.putBoolean( "NO LOOP DELAY PROBLEM", noLoopDelayProblem );
      sendUpdate = false;
    }
    lastTime = Timer.getFPGATimestamp();
    
    return noLoopDelayProblem;
  }  // end lookForTimeTrouble

} // end class Robot
