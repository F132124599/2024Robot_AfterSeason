// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CANdleConfiguration candleConfig;
  private final int ledNum;
  private Animation ledAnimation;

  public interface InnerLEDSubsystem {
     boolean LEDFlag = false;
     boolean hasNote = false;
     boolean intaking = false;
     boolean trackingNote = false;
     boolean hasNoteInSight = false;
     boolean prepSPEAKER = false;
     boolean prepAMP = false;
     boolean speedReadySPEAKER = false;
     boolean speedReadyAMP = false;
     boolean aimingAMP = true;
     boolean aimReadyAMP = false;
     boolean haveApriltag = true ;
     boolean playing = false;
     boolean prepPassNote = false;
     boolean speedReadyPassNote = false;

  }

  public LEDSubsystem() {
    candle = new CANdle(LEDConstants.candle_ID);

    ledNum = LEDConstants.LedNum;
    candleConfig = new CANdleConfiguration();
    candleConfig.stripType = LEDStripType.RGB;
    candleConfig.statusLedOffWhenActive = true;
    candleConfig.disableWhenLOS = false;
    candleConfig.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfig);

    ledAnimation = null;
  }
  /* =========
   *   Blink
   * =========*/
  public void redBlink(){
    ledAnimation = new StrobeAnimation(255, 0, 0, 0, 0.4, ledNum);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void blueBlink(){
    ledAnimation = new StrobeAnimation(0, 0, 255, 0, 0.4, ledNum);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void yellowBlink(){
    ledAnimation = new StrobeAnimation(255, 100, 0, 0, 0.4, ledNum);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void purpleBlink(){
    ledAnimation = new StrobeAnimation(255, 0, 255, 0, 0.4, ledNum);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  public void pinkBlink(){
    ledAnimation = new StrobeAnimation(77, 0, 9, 0, 0.4, ledNum);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }
  /* =========
   *   Solid
   * =========*/
  public void redSolid(){
    candle.animate(null);
    candle.setLEDs(255, 0, 0);
    LEDConstants.LEDFlag = false;
  }
  public void greenSolid(){
    candle.animate(null);
    candle.setLEDs(0, 255, 0);
    LEDConstants.LEDFlag = false;
  }
  public void blueSolid(){
    candle.animate(null);
    candle.setLEDs(0, 0, 255);
    LEDConstants.LEDFlag = false;
  }
  public void purpleSolid(){
    candle.animate(null);
    candle.setLEDs(255, 0, 255);
    LEDConstants.LEDFlag = false;
  }
  public void stopLED(){
    candle.animate(null);
    candle.setLEDs(0, 0, 0);
    LEDConstants.LEDFlag = false;
  }
  public void pinkSolid(){
    candle.animate(null);
    candle.setLEDs(77, 0, 9);
    LEDConstants.LEDFlag = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LEDConstants.LEDFlag){
      if(LEDConstants.playing == false) stopLED();
      else if(LEDConstants.speedReadyAMP) blueSolid();
      else if(LEDConstants.speedReadySPEAKER) purpleSolid();
      else if(LEDConstants.speedReadyPassNote) pinkSolid();
      else if(LEDConstants.prepSPEAKER) purpleBlink();
      else if(LEDConstants.prepAMP) blueBlink();
      else if(LEDConstants.prepPassNote) pinkBlink();
      else if(LEDConstants.hasNoteInSight && LEDConstants.trackingNote) yellowBlink();
      else if(LEDConstants.hasNoteInSight == false && LEDConstants.trackingNote) redBlink();
      else if(LEDConstants.intaking) redBlink();
      else if(LEDConstants.hasNote) greenSolid();
      else if(LEDConstants.hasNote == false) redSolid();
      LEDConstants.LEDFlag = false;
    }    
  }
}
