package org.usfirst.frc.team3853.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

  private SpeedController motor;
  private Encoder encoder;
  private DigitalInput limit;

  enum MotorDir {
    CW(-1.0), CCW(1.0);
    private double value;

    private MotorDir(double value) {
      this.value = value;
    }

    double v() {
      return this.value;
    }
  }

  private MotorDir md; // -1 = CW 1 = CCW

  private double speed; // eventually, get this from SmartDashboard
  private boolean lastToggleDir; // edge detector on direction switch

  public Arm(int pwm, int encA, int encB, int limitSw) {

    // create the parts
    motor = new Talon( pwm );
    LiveWindow.addActuator( "Arm", "ArmMotor", (Talon) motor );
    encoder = new Encoder( encA, encB, true, EncodingType.k4X );
    LiveWindow.addSensor( "Arm", "ArmEncoder", encoder );
    limit = new DigitalInput( limitSw );
    LiveWindow.addSensor( "Arm", "ArmLimit", limit );

    // arm motor control
    motor.stopMotor();
    speed = 0.3;
    md = MotorDir.CW;
    
    // initialize edge detector for the direction button
    lastToggleDir = false;

    encoder.reset();
    encoder.setDistancePerPulse( 1.0 );
    encoder.setPIDSourceType( PIDSourceType.kDisplacement );

  } // end Arm ctor

  public void go(boolean move, boolean toggleDir) {

    // detect press on direction button
    if ( toggleDir && !lastToggleDir ) {
      lastToggleDir = true;
      switch ( md ) {
        case CW:
          md = MotorDir.CCW;
        case CCW:
          md = MotorDir.CW;
      }
    }

    if ( move ) {
      // stop the arm motor on the limit switch
      if ( limit.get() && md == MotorDir.CCW ) {
        md = MotorDir.CW;
        motor.stopMotor();
        encoder.reset();
      } else {
        motor.set( speed * md.v() );
      }
    } else {
      motor.stopMotor();
    }

  } // end go arm movement control

  public void stop() {
    motor.stopMotor();
  }

  public void sendAll() {
    SmartDashboard.putNumber( "ARM distance", encoder.getDistance() );
    SmartDashboard.putNumber( "ARM rate", encoder.getRate() );
    SmartDashboard.putNumber( "ARM raw", encoder.getRaw() );
    SmartDashboard.putBoolean( "ARM stopped", encoder.getStopped() );
    SmartDashboard.putBoolean( "ARM limit", limit.get() );
  } // end method sendAll

} // end class Arm
