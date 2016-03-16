package org.usfirst.frc.team3853.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase {

  private SpeedController lmc, rmc;
  private RobotDrive dt; // class that handles basic drive operations

  enum DriveStyle {
    Tank, Arcade
  }

  private DriveStyle ds;

  public DriveBase(int leftPwm, int rightPwm) {
    lmc = new Talon( leftPwm );
    rmc = new Talon( rightPwm );
    dt = new RobotDrive( lmc, rmc );
    dt.setExpiration( 0.1 );
    dt.stopMotor();

    LiveWindow.addActuator( "DriveBase", "Lmc", (Talon) lmc );
    LiveWindow.addActuator( "DriveBase", "Rmc", (Talon) rmc );
  } // end DiveBase ctor

  // screeching halt
  public void stopNow() {
    dt.stopMotor();
  }

  // tank drive movement from joystck
  public void go(Joystick l, Joystick r, boolean square) {
    ds = DriveStyle.Tank;
    dt.tankDrive( l, r, square );
  }

  // tank drive movement by value
  public void go(double l, double r, boolean tankORarcade) {
    if ( tankORarcade ) {
      ds = DriveStyle.Tank;
      dt.tankDrive( l, r );
    } else {
      ds = DriveStyle.Arcade;
      dt.arcadeDrive( l, r );
    }
  }

  // arcade drive movement joystick
  public void go(Joystick j, boolean square) {
    ds = DriveStyle.Arcade;
    dt.arcadeDrive( j, square );
  }

  public void sendAll() {
    SmartDashboard.putString( "Drive style", ds.toString() );
  }
} // end class DriveBase
