package org.usfirst.frc.team3853.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ball {

  private SpeedController motor;
  private DigitalInput limit;

  private double InSpeed;
  private double BarfSpeed;
  private double PukeSpeed;
  private double TossTime;

  public Ball(int pwm, int limitID) {

    // motor control
    InSpeed   = -0.3 ;
    BarfSpeed =  0.3 ;
    PukeSpeed =  0.6 ;
    TossTime  =  0.2 ;

    motor = new Talon( pwm );
    LiveWindow.addActuator( "Ball", "motor", (Talon) motor );

    limit = new DigitalInput( limitID );
    LiveWindow.addSensor( "BallLimit", "limit", limit );

    stop();
  }

  public void go(boolean takeIn, boolean pukeOut, boolean barfOut) {
    if ( limit.get() && InSpeed < 0.0 ) { // this depends on motor direction
      stop(); // stop the world I have to get off
    } else {
      if ( takeIn ) {
        set( InSpeed );
      } else if ( barfOut ) {
        set( BarfSpeed );
      } else if ( pukeOut ) {
        set( PukeSpeed );
      } else {
        stop(); // stop the world I want to get off
      }
    }
  } // end go

  public void set ( double speed ) {
    motor.set( speed );
  }
  
  public void stop() {
    motor.stopMotor();
  }

  public void sendAll() {
    SmartDashboard.putBoolean( "BALL limit", limit.get() );
    SmartDashboard.putNumber ( "BALL speed", motor.get() );
  }
} // end class Ball