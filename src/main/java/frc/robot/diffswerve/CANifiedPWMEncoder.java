package frc.robot.diffswerve;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifierStatusFrame;

/** Represents a PWM-based encoder that is connected to a CANifier. */
public class CANifiedPWMEncoder {

  // The CANifier that the encoder is plugged into
  private CANifier canifier;

  // The channel that the encoder is plugged into
  private PWMChannel channel;

  // Conversion from ticks to number of rotations
  private double ratio = 1.0;

  // offset between encoder zero and robot zero (radians)
  private double offset = 0.0;

  // whether encoder is flipped
  private boolean inverted = false;

  private double prevAngle = 0.0;

  /**
   * Constructor.
   *
   * @param canifier The CANifier that the encoder is plugged into
   * @param channel The channel that the encoder is plugged into
   */
  public CANifiedPWMEncoder(int canifierID, int pwmChannel, double offset, double ratio, boolean inverted) {
    this.ratio = ratio;
    this.offset = offset;
    this.inverted = inverted;

    PWMChannel channel;
    switch (pwmChannel) {
      case 0:
        channel = PWMChannel.PWMChannel0;
        break;
      case 1:
        channel = PWMChannel.PWMChannel1;
        break;
      case 2:
        channel = PWMChannel.PWMChannel2;
        break;
      case 3:
        channel = PWMChannel.PWMChannel3;
        break;
      default:
        throw new RuntimeException(
            String.format(
                "%d is not a valid PWM channel. It must be between 0 and 3, inclusive",
                pwmChannel));
    }

    this.canifier = new CANifier(canifierID);
    this.channel = channel;
    // Set update rate of sensors in milliseconds
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 5);
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  // Returns position in radians
  public double getPosition() {
    double[] dutyAndPeriod = new double[2];
    this.canifier.getPWMInput(channel, dutyAndPeriod);
    if (dutyAndPeriod[1] == 0.0) {
      // Sensor is unplugged
      System.out.println(String.format("CANified PWM Channel %d is disconnected!", channel.value));
      return prevAngle;
    }
    prevAngle = this.inverted ? -1.0 : 1.0 * this.ratio * dutyAndPeriod[0] / dutyAndPeriod[1] - this.offset;
    return prevAngle;
  }
}
