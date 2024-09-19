package frc.robot.extras;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.Constants.HardwareConstants;

public class CANTHINGY {
  public static boolean isCANFD = false;

  public static boolean isCANFD(ParentDevice device) {return isCANFD = CANBus.isNetworkFD(device.getNetwork());  }

  public static String getCANBus(ParentDevice device) {
    if (isCANFD(device)) {
      return DeviceCANBus.CANIVORE.name;
    } return DeviceCANBus.RIO.name;
  }
  /** CTRE Phoenix CAN bus */
  public enum DeviceCANBus {
    /** roboRIO CAN bus */
    RIO(HardwareConstants.RIO_CAN_BUS_STRING),
    /**
     * CANivore CAN bus
     * <p>
     * Only a single CANivore is supported, and MUST be named "canivore"
     */
    CANIVORE(HardwareConstants.CANIVORE_CAN_BUS_STRING);

    /** CAN bus name */
    public final String name;
    private DeviceCANBus(String name) {
      this.name = name;
    }
  }
}
