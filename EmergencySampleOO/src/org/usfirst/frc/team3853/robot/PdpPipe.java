package org.usfirst.frc.team3853.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PdpPipe extends PowerDistributionPanel {

  private final static int maxTop = 31;
  private int[] pwmList;
  private int top;

  public PdpPipe() {

    this.resetTotalEnergy();

    pwmList = new int[maxTop];
    top = 0;
  }

  public boolean addPwm(int pwmNum) {
    if ( top <= maxTop ) {
      pwmList[top] = pwmNum;
      top += 1;
      return true;
    } else {
      return false;
    }
  }

  public void sendVolts() {
    SmartDashboard.putNumber( "PDP Volts", getVoltage() );
  }

  public void sendAmps() {
    SmartDashboard.putNumber( "PDP Total Amps", getTotalCurrent() );
  }

  public void sendPwmAmps() {
    for ( int i = 0; i <= maxTop; i += 1 ) {
      SmartDashboard.putNumber( String.format( "PDP Amps pwm %d", i ), getCurrent( i ) );
    }
  }

  public void sendAll() {
    sendVolts();
    sendAmps();
    sendPwmAmps();
  }
}
