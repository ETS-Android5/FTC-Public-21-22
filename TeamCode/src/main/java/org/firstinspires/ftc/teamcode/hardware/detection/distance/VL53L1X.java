package org.firstinspires.ftc.teamcode.hardware.detection.distance;

import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(xmlTag = "VL53L1X", name = "VL53L1X", description = "A 4m Distance Sensor")
public class VL53L1X extends I2cDeviceSynchDevice<I2cDeviceSynch> implements IVL53L1X {
  public VL53L1X(I2cDeviceSynch i2cDeviceSynch) {
    super(i2cDeviceSynch, true);
    this.deviceClient.setI2cAddress(
        I2cAddr.create8bit(
            VL53L1X_Constants.VL53L1_I2C_SLAVE__DEVICE_ADDRESS)); // Maybe comment this out?
    super.registerArmingStateCallback(false);
    this.deviceClient.engage();
  }

  private boolean calibrated = false;

  @Override
  protected boolean doInitialize() {
    if (!calibrated) {
      calibrate();
    }
    VL53L1X_StartRanging();
    return true;
  }

  @Override
  public Manufacturer getManufacturer() {
    return Manufacturer.STMicroelectronics;
  }

  @Override
  public String getDeviceName() {
    return "STMicroelectronics_VL53L1X_Range_Sensor";
  }

  // INITIALIZE MUST BE CALLED FIRST
  //
  @Override
  public double getDistance(DistanceUnit unit) {
    Log.d("VL53L1X", "GETTING DISTANCE");
    byte dataReady = 0;
    while (dataReady == 0) {
      dataReady = VL53L1X_CheckForDataReady();
      Log.d("VL53L1X", "CHECKING FOR DATA READY: " + dataReady);
    }
    Log.d("VL53L1X", "CHECKING RANGE STATUS");
    byte rangeStatus = VL53L1X_GetRangeStatus();
    Log.d("VL53L1X", "RANGE STATUS: " + rangeStatus);
    short distance = VL53L1X_GetDistance();
    Log.d("VL53L1X", "DISTANCE: " + distance);
    VL53L1X_ClearInterrupt();
    Log.d("VL53L1X", "INTERRUPT CLEARED");
    return distance;
  }

  // MUST BE CALIBRATED USING 17% REFLECTANCE GRAY SHEET AT 140MM
  @Override
  public void calibrate() {
    Log.d("VL53L1X", "CALIBRATION");
    byte bootState = 0;
    while (VL53L1X_BootState() == 0) {
      bootState = VL53L1X_BootState();
      Log.d("VL53L1X", "BOOT STATE: " + bootState);
    }
    Log.d("VL53L1X", "BOOTED");
    VL53L1X_SensorInit();
    Log.d("VL53L1X", "INITIALIZED");
    // This calibrates it to 100mm
    short offset = VL53L1X_CalibrateOffset((short) 140);
    Log.d("VL53L1X", "OFFSET CALIBRATED");
    Log.d("VL53L1X", "OFFSET: " + offset);
    short xtalk = VL53L1X_CalibrateXtalk((short) 140);
    Log.d("VL53L1X", "XTALK CALIBRATED");
    Log.d("VL53L1X", "XTALK: " + xtalk);
    calibrated = true;
  }

  /*
  NOTE: Timing budget options in ms are
  20, 33, 50, 100, 200, 500
   */
  @Override
  public void setTimingBudget(short budgetMs) {
    Log.d("VL53L1X", "SETTING TIMING BUDGET");
    VL53L1X_SetTimingBudgetInMs(budgetMs);
    Log.d("VL53L1X", "SETTING INTER-MEASUREMENT PERIOD");
    VL53L1X_SetInterMeasurementInMs(budgetMs * 2);
  }

  @Override
  public void onShutdown() {
    Log.d("VL53L1X", "SHUTTING DOWN");
    VL53L1X_StopRanging();
  }

  public VL53L1X_Version VL53L1X_GetSWVersion() {
    return new VL53L1X_Version(
            VL53L1X_Constants.VL53L1X_IMPLEMENTATION_VER_MAJOR,
            VL53L1X_Constants.VL53L1X_IMPLEMENTATION_VER_MINOR,
            VL53L1X_Constants.VL53L1X_IMPLEMENTATION_VER_SUB,
            VL53L1X_Constants.VL53L1X_IMPLEMENTATION_VER_REVISION
    );
  }

  public void VL53L1X_SetI2CAddress(byte new_address) {
    deviceClient.write8(VL53L1X_Constants.VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
  }

  public void VL53L1X_SensorInit() {
    for (short Addr = 0x2D; Addr <= 0x87; Addr++) {
      deviceClient.write8(Addr, VL53L1X_Constants.VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
    }
    VL53L1X_StartRanging();
    byte tmp = 0;
    while (tmp == 0) {
      tmp = VL53L1X_CheckForDataReady();
    }
    VL53L1X_ClearInterrupt();
    VL53L1X_StopRanging();
    deviceClient.write8(VL53L1X_Constants.VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    deviceClient.write8(0x0B, 0);
  }

  public void VL53L1X_ClearInterrupt() {
    deviceClient.write8(VL53L1X_Constants.SYSTEM__INTERRUPT_CLEAR, 0x01);
  }

  void VL53L1X_SetInterruptPolarity(byte NewPolarity) {
      byte temp = deviceClient.read8(VL53L1X_Constants.GPIO_HV_MUX__CTRL);
      temp &= 0xEF;
      deviceClient.write8(VL53L1X_Constants.GPIO_HV_MUX__CTRL, (temp | (((NewPolarity & 1) == 0 ? 1 : 0)) << 4));
  }

  byte VL53L1X_GetInterruptPolarity() {
    byte temp = deviceClient.read8(VL53L1X_Constants.GPIO_HV_MUX__CTRL);
    temp &= 0x10;
    return (byte) ((temp >> 4) == 0 ? 1 : 0);
  }

  void VL53L1X_StartRanging() {
    deviceClient.write8(VL53L1X_Constants.SYSTEM__MODE_START, 0x40);
  }

  void VL53L1X_StopRanging() {
    deviceClient.write8(VL53L1X_Constants.SYSTEM__MODE_START, 0x00);
  }

  byte VL53L1X_CheckForDataReady() {
    byte IntPol = VL53L1X_GetInterruptPolarity();
    byte temp = deviceClient.read8(VL53L1X_Constants.GPIO__TIO_HV_STATUS);
    return (byte) ((temp & 1) == IntPol ? 1 : 0);
  }

  void VL53L1X_SetTimingBudgetInMs(short TimingBudgetInMs) {
    short DM = VL53L1X_GetDistanceMode();
    if (DM == 0) return;
    if (DM == 1) {
      switch (TimingBudgetInMs) {
        case 15:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x01D));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x0027));
          break;
        case 20:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x0051));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x006E));
          break;
        case 33:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x00D6));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x006E));
          break;
        case 50:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x1AE));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x01E8));
          break;
        case 100:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x02E1));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x0388));
          break;
        case 200:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x03E1));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x0496));
          break;
        case 500:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x0591));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x05C1));
          break;
      }
    } else {
      switch (TimingBudgetInMs) {
        case 20:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x001E));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x0022));
          break;
        case 33:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x0060));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x006E));
          break;
        case 50:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x00AD));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x00C6));
          break;
        case 100:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x01CC));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x01EA));
          break;
        case 200:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x02D9));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x02F8));
          break;
        case 500:
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TypeConversion.shortToByteArray((short) 0x048F));
          deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TypeConversion.shortToByteArray((short) 0x04A4));
          break;
      }
    }
  }

  short VL53L1X_GetTimingBudgetInMs() {
    short Temp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.RANGE_CONFIG__TIMEOUT_MACROP_A_HI));
    switch (Temp) {
      case 0x001D:
        return 15;
      case 0x0051:
      case 0x001E:
        return 20;
      case 0x00D6:
      case 0x0060:
        return 33;
      case 0x1AE:
      case 0x00AD:
        return 50;
      case 0x02E1:
      case 0x01CC:
        return 100;
      case 0x03E1:
      case 0x02D9:
        return 200;
      case 0x0591:
      case 0x048F:
        return 500;
      default:
        return 0;
    }
  }

  void VL53L1X_SetDistanceMode(short DM) {
    short TB = VL53L1X_GetTimingBudgetInMs();
    if (DM == 1) {
      deviceClient.write8(VL53L1X_Constants.PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      deviceClient.write(VL53L1X_Constants.SD_CONFIG__WOI_SD0, TypeConversion.shortToByteArray((short) 0x0705));
      deviceClient.write(VL53L1X_Constants.SD_CONFIG__INITIAL_PHASE_SD0, TypeConversion.shortToByteArray((short) 0x0606));
    } else if (DM == 2) {
      deviceClient.write8(VL53L1X_Constants.PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      deviceClient.write8(VL53L1X_Constants.RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
      deviceClient.write(VL53L1X_Constants.SD_CONFIG__WOI_SD0, TypeConversion.shortToByteArray((short) 0x0F0D));
      deviceClient.write(VL53L1X_Constants.SD_CONFIG__INITIAL_PHASE_SD0, TypeConversion.shortToByteArray((short) 0x0E0E));
    }
    VL53L1X_SetTimingBudgetInMs(TB);
  }

  short VL53L1X_GetDistanceMode() {
    switch (deviceClient.read8(VL53L1X_Constants.PHASECAL_CONFIG__TIMEOUT_MACROP)) {
      case 0x14:
        return 1;
      case 0x0A:
        return 2;
      default:
        return 0;
    }
  }

  void VL53L1X_SetInterMeasurementInMs(int InterMeasMs) {
    short ClockPLL = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__OSC_CALIBRATE_VAL));
    ClockPLL &= 0x3FF;
    deviceClient.write(VL53L1X_Constants.VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, TypeConversion.intToByteArray((int) (ClockPLL * InterMeasMs * 1.075)));
  }

  short VL53L1X_GetInterMeasurementInMs() {
    int tmp = TypeConversion.byteArrayToInt(deviceClient.read(VL53L1X_Constants.VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD));
    short ClockPLL = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__OSC_CALIBRATE_VAL));
    ClockPLL &= 0x3FF;
    tmp /= ClockPLL * 1.065;
    return (short) tmp;
  }

  byte VL53L1X_BootState() {
    return deviceClient.read8(VL53L1X_Constants.VL53L1_FIRMWARE__SYSTEM_STATUS);
  }

  short VL53L1X_GetSensorId() {
    return TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_IDENTIFICATION__MODEL_ID));
  }

  short VL53L1X_GetDistance() {
    return TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
  }

  short VL53L1X_GetSignalPerSpad() {
    short signal = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0));
    short SpNb = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0));
    return (short) (200.0 * signal / SpNb);
  }

  short VL53L1X_GetAmbientPerSpad() {
    short AmbientRate = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.RESULT__AMBIENT_COUNT_RATE_MCPS_SD));
    short SpNb = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0));
    return (short) (200.0 * AmbientRate / SpNb);
  }

  short VL53L1X_GetSignalRate() {
    return (short) (TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0)) * 8);
  }

  short VL53L1X_GetSpadNb() {
    short tmp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0));
    return (short) (tmp >> 8);
  }

  short VL53L1X_GetAmbientRate() {
    short tmp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.RESULT__AMBIENT_COUNT_RATE_MCPS_SD));
    return (short) (tmp * 8);
  }

  byte VL53L1X_GetRangeStatus() {
    byte RgSt = deviceClient.read8(VL53L1X_Constants.VL53L1_RESULT__RANGE_STATUS);
    RgSt &= 0x1F;
    if (RgSt < 24) {
      return VL53L1X_Constants.status_rtn[(RgSt & 0xFF)];
    } else {
      return (byte) 255;
    }
  }

  VL53L1X_Result VL53L1X_GetResult() {
    byte[] Temp = deviceClient.read(VL53L1X_Constants.VL53L1_RESULT__RANGE_STATUS, 17);
    byte RgSt = (byte) (Temp[0] & 0x1F);
    if (RgSt < 24) RgSt = VL53L1X_Constants.status_rtn[(RgSt & 0xFF)];
    return new VL53L1X_Result(
            RgSt,
            TypeConversion.byteArrayToShort(new byte[] { Temp[13], Temp[14] }),
            (short) (TypeConversion.byteArrayToShort(new byte[] { Temp[7], Temp[8] }) * 8),
            (short) (TypeConversion.byteArrayToShort(new byte[] { Temp[15], Temp[16] }) * 8),
            Temp[3]
    );
  }

  void VL53L1X_SetOffset(short OffsetValue) {
    short Temp = (short) (OffsetValue * 4);
    deviceClient.write(VL53L1X_Constants.ALGO__PART_TO_PART_RANGE_OFFSET_MM, TypeConversion.shortToByteArray(Temp));
    deviceClient.write(VL53L1X_Constants.MM_CONFIG__INNER_OFFSET_MM, TypeConversion.shortToByteArray((short) 0));
    deviceClient.write(VL53L1X_Constants.MM_CONFIG__OUTER_OFFSET_MM, TypeConversion.shortToByteArray((short) 0));
  }

  short VL53L1X_GetOffset() {
    short temp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.ALGO__PART_TO_PART_RANGE_OFFSET_MM));
    temp <<= 3;
    temp >>= 5;
    return temp;
  }

  void VL53L1X_SetXtalk(short XtalkValue) {
    deviceClient.write(VL53L1X_Constants.ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, TypeConversion.shortToByteArray((short) 0));
    deviceClient.write(VL53L1X_Constants.ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, TypeConversion.shortToByteArray((short) 0));
    deviceClient.write(VL53L1X_Constants.ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, TypeConversion.shortToByteArray((short) ((XtalkValue << 9) / 1000)));
  }

  short VL53L1X_GetXtalk() {
    short xtalk = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS));
    return (short) ((xtalk * 1000) >> 9);
  }

  void VL53L1X_SetDistanceThreshold(short ThreshLow, short ThreshHigh, byte Window, byte IntOnNoTarget) {
    byte temp = deviceClient.read8(VL53L1X_Constants.SYSTEM__INTERRUPT_CONFIG_GPIO);
    temp &= 0x47;
    if (IntOnNoTarget == 0) {
      deviceClient.write8(VL53L1X_Constants.SYSTEM__INTERRUPT_CONFIG_GPIO, (temp | (Window & 0x07)));
    } else {
      deviceClient.write8(VL53L1X_Constants.SYSTEM__INTERRUPT_CONFIG_GPIO, ((temp | (Window & 0x07)) | 0x40));
    }
    deviceClient.write(VL53L1X_Constants.SYSTEM__THRESH_HIGH, TypeConversion.shortToByteArray(ThreshHigh));
    deviceClient.write(VL53L1X_Constants.SYSTEM__THRESH_LOW, TypeConversion.shortToByteArray(ThreshLow));
  }

  short VL53L1X_GetDistanceThresholdWindow() {
    return (short) (deviceClient.read8(VL53L1X_Constants.SYSTEM__INTERRUPT_CONFIG_GPIO) & 0x7);
  }

  short VL53L1X_GetDistanceThresholdLow() {
    return TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.SYSTEM__THRESH_LOW));
  }

  short VL53L1X_GetDistanceThresholdHigh() {
    return TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.SYSTEM__THRESH_HIGH));
  }

  void VL53L1X_SetROICenter(byte ROICenter) {
    deviceClient.write8(VL53L1X_Constants.ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
  }

  byte VL53L1X_GetROICenter() {
    return deviceClient.read8(VL53L1X_Constants.ROI_CONFIG__USER_ROI_CENTRE_SPAD);
  }

  void VL53L1X_SetROI(short X, short Y) {
    byte OpticalCenter = deviceClient.read8(VL53L1X_Constants.VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD);
    if (X > 16) X = 16;
    if (Y > 16) Y = 16;
    if (X > 10 || Y > 10) {
      OpticalCenter = (byte) 199;
    }
    deviceClient.write8(VL53L1X_Constants.ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
    deviceClient.write8(VL53L1X_Constants.ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (Y - 1) << 4 | (X - 1));
  }

  Pair<Short, Short> VL53L1X_GetROI_XY() {
    byte tmp = deviceClient.read8(VL53L1X_Constants.ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
    short ROI_X = (short) (((short) tmp & 0x0F) + 1);
    short ROI_Y = (short) ((((short) tmp & 0xF0) >> 4) + 1);
    return new Pair<>(ROI_X, ROI_Y);
  }

  void VL53L1X_SetSignalThreshold(short Signal) {
    deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, TypeConversion.shortToByteArray(Signal));
  }

  short VL53L1X_GetSignalThreshold() {
    short tmp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS));
    return (short) (tmp << 3);
  }

  void VL53L1X_SetSigmaThreshold(short Sigma) {
    if (Sigma > (0xFFFF >> 2)) {
      return;
    }
    deviceClient.write(VL53L1X_Constants.RANGE_CONFIG__SIGMA_THRESH, TypeConversion.shortToByteArray((short) (Sigma << 2)));
  }

  short VL53L1X_GetSigmaThreshold() {
    short tmp = TypeConversion.byteArrayToShort(deviceClient.read(VL53L1X_Constants.RANGE_CONFIG__SIGMA_THRESH));
    return (short) (tmp >> 2);
  }

  void VL53L1X_StartTemperatureUpdate() {
    byte tmp = 0;
    deviceClient.write8(VL53L1X_Constants.VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81);
    deviceClient.write8(0x0B, 0x92);
    VL53L1X_StartRanging();
    while (tmp == 0) {
      tmp = VL53L1X_CheckForDataReady();
    }
    VL53L1X_ClearInterrupt();
    VL53L1X_StopRanging();
    deviceClient.write8(VL53L1X_Constants.VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    deviceClient.write8(0x0B, 0);
  }

  short VL53L1X_CalibrateOffset(short TargetDistInMm) {
    byte i, tmp;
    short AverageDistance = 0;
    short distance;

    deviceClient.write(VL53L1X_Constants.ALGO__PART_TO_PART_RANGE_OFFSET_MM, TypeConversion.shortToByteArray((short) 0x0));
    deviceClient.write(VL53L1X_Constants.MM_CONFIG__INNER_OFFSET_MM, TypeConversion.shortToByteArray((short) 0x0));
    VL53L1X_StartRanging();	/* Enable VL53L1X sensor */
    for (i = 0; i < 50; i++) {
      tmp = 0;
      while (tmp == 0) {
        tmp = VL53L1X_CheckForDataReady();
      }
      distance = VL53L1X_GetDistance();
      VL53L1X_ClearInterrupt();
      AverageDistance += distance;
    }
    VL53L1X_StopRanging();
    AverageDistance /= 50;
	short offset = (short) (TargetDistInMm - AverageDistance);
	deviceClient.write(VL53L1X_Constants.ALGO__PART_TO_PART_RANGE_OFFSET_MM, TypeConversion.shortToByteArray((short) (offset * 4)));
    return offset;
  }

  short VL53L1X_CalibrateXtalk(short TargetDistInMm) {
    byte i, tmp;
    float AverageSignalRate = 0;
    float AverageDistance = 0;
    float AverageSpadNb = 0;
    short distance = 0, spadNum;
    short sr;
    int calXtalk;

    deviceClient.write(0x0016, TypeConversion.shortToByteArray((short) 0));
    VL53L1X_StartRanging();
    for (i = 0; i < 50; i++) {
      tmp = 0;
      while (tmp == 0){
        tmp = VL53L1X_CheckForDataReady();
      }
      sr = VL53L1X_GetSignalRate();
      distance = VL53L1X_GetDistance();
      VL53L1X_ClearInterrupt();
      AverageDistance += distance;
      spadNum = VL53L1X_GetSpadNb();
      AverageSpadNb += spadNum;
      AverageSignalRate += sr;
    }
    VL53L1X_StopRanging();
    AverageDistance /= 50;
    AverageSpadNb /= 50;
    AverageSignalRate /= 50;
    /* Calculate Xtalk value */
    calXtalk = (short) (512 * (AverageSignalRate * (1 - (AverageDistance / TargetDistInMm))) / AverageSpadNb);
    if (calXtalk  > (short) 0xffff) calXtalk = 0xffff;
    deviceClient.write(0x0016, TypeConversion.shortToByteArray((short) calXtalk));
    return (short) ((calXtalk * 1000) >> 9);
  }
}
