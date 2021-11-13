package org.firstinspires.ftc.teamcode.hardware.detection.distance;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@I2cDeviceType
//@DeviceProperties(xmlTag = "VL53L1X", name = "VL53L1X", description = "A 4m Distance Sensor")
public class VL53L1X extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    public VL53L1X(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    @Override
    protected boolean doInitialize() {
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

    @Override
    public double getDistance(DistanceUnit unit) {
        return 0;
    }


    public byte VL53L1X_GetSWVersion(VL53L1X_Version_t pVersion)
    {
        byte Status = 0;

        pVersion.major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
        pVersion.minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
        pVersion.build = VL53L1X_IMPLEMENTATION_VER_SUB;
        pVersion.revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
        return Status;
    }

    public byte VL53L1X_SetI2CAddress(int dev, short new_address)
    {
        byte status = 0;

        status |= VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
        return status;
    }

    public byte VL53L1X_SensorInit(int dev)
    {
        byte status = 0;
        short Addr = 0x00, tmp;

        for (Addr = 0x2D; Addr <= 0x87; Addr++) {
            status |= VL53L1_WrByte(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
        }
        status |= VL53L1X_StartRanging(dev);
        tmp  = 0;
        while(tmp==0){
            status |= VL53L1X_CheckForDataReady(dev, &tmp);
        }
        status |= VL53L1X_ClearInterrupt(dev);
        status |= VL53L1X_StopRanging(dev);
        status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
        status |= VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
        return status;
    }

    public byte VL53L1X_ClearInterrupt(int dev)
    {
        byte status = 0;

        status |= VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
        return status;
    }

    byte VL53L1X_SetInterruptPolarity(int dev, short NewPolarity)
    {
        short Temp;
        byte status = 0;

        status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, Temp);
        Temp = Temp & 0xEF;
        status |= VL53L1_WrByte(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
        return status;
    }

    byte VL53L1X_GetInterruptPolarity(int dev, short pInterruptPolarity)
    {
        short Temp;
        byte status = 0;

        status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, Temp);
        Temp = Temp & 0x10;
	pInterruptPolarity = !(Temp>>4);
        return status;
    }

    byte VL53L1X_StartRanging(int dev)
    {
        byte status = 0;

        status |= VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);	/* Enable VL53L1X */
        return status;
    }

    byte VL53L1X_StopRanging(int dev)
    {
        byte status = 0;

        status |= VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00);	/* Disable VL53L1X */
        return status;
    }

    byte VL53L1X_CheckForDataReady(int dev, short isDataReady)
    {
        short Temp;
        short IntPol;
        byte status = 0;

        status |= VL53L1X_GetInterruptPolarity(dev, IntPol);
        status |= VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, Temp);
        /* Read in the register to check if a new value is available */
        if (status == 0){
            if ((Temp & 1) == IntPol)
			isDataReady = 1;
		else
			isDataReady = 0;
        }
        return status;
    }

    byte VL53L1X_SetTimingBudgetInMs(int dev, int TimingBudgetInMs)
    {
        int DM;
        byte  status=0;

        status |= VL53L1X_GetDistanceMode(dev, DM);
        if (DM == 0)
            return 1;
        else if (DM == 1) {	/* Short DistanceMode */
            switch (TimingBudgetInMs) {
                case 15: /* only available in short distance mode */
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x01D);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x0027);
                    break;
                case 20:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x0051);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x006E);
                    break;
                case 33:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x00D6);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x006E);
                    break;
                case 50:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x1AE);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x01E8);
                    break;
                case 100:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x02E1);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x0388);
                    break;
                case 200:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x03E1);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x0496);
                    break;
                case 500:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x0591);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x05C1);
                    break;
                default:
                    status = 1;
                    break;
            }
        } else {
            switch (TimingBudgetInMs) {
                case 20:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x001E);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x0022);
                    break;
                case 33:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x0060);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x006E);
                    break;
                case 50:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x00AD);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x00C6);
                    break;
                case 100:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x01CC);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x01EA);
                    break;
                case 200:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x02D9);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x02F8);
                    break;
                case 500:
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                            0x048F);
                    VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                            0x04A4);
                    break;
                default:
                    status = 1;
                    break;
            }
        }
        return status;
    }

    byte VL53L1X_GetTimingBudgetInMs(int dev, int pTimingBudget)
    {
        int Temp;
        byte status = 0;

        status |= VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, Temp);
        switch (Temp) {
            case 0x001D :
			pTimingBudget = 15;
                break;
            case 0x0051 :
            case 0x001E :
			pTimingBudget = 20;
                break;
            case 0x00D6 :
            case 0x0060 :
			pTimingBudget = 33;
                break;
            case 0x1AE :
            case 0x00AD :
			pTimingBudget = 50;
                break;
            case 0x02E1 :
            case 0x01CC :
			pTimingBudget = 100;
                break;
            case 0x03E1 :
            case 0x02D9 :
			pTimingBudget = 200;
                break;
            case 0x0591 :
            case 0x048F :
			pTimingBudget = 500;
                break;
            default:
                status = 1;
			pTimingBudget = 0;
        }
        return status;
    }

    byte VL53L1X_SetDistanceMode(int dev, int DM)
    {
        int TB;
        byte status = 0;

        status |= VL53L1X_GetTimingBudgetInMs(dev, &TB);
        if (status != 0)
            return 1;
        switch (DM) {
            case 1:
                status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
                status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0705);
                status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
                break;
            case 2:
                status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
                status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
                status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
                status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
                break;
            default:
                status = 1;
                break;
        }

        if (status == 0)
            status |= VL53L1X_SetTimingBudgetInMs(dev, TB);
        return status;
    }

    byte VL53L1X_GetDistanceMode(int dev, int DM)
    {
        short TempDM;
        byte status=0;

        status |= VL53L1_RdByte(dev,PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
        if (TempDM == 0x14){
            DM=1;
        }
		    
        if(TempDM == 0x0A) {
            DM=2;
        }
        return status;
    }

    byte VL53L1X_SetInterMeasurementInMs(int dev, long InterMeasMs)
    {
        int ClockPLL;
        byte status = 0;

        status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, ClockPLL);
        ClockPLL = ClockPLL&0x3FF;
        VL53L1_WrDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
                (long)(ClockPLL * InterMeasMs * 1.075));
        return status;

    }

    byte VL53L1X_GetInterMeasurementInMs(int dev, int pIM)
    {
        int ClockPLL;
        byte status = 0;
        long tmp;

        status |= VL53L1_RdDWord(dev,VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, tmp);
	pIM = (int)tmp;
        status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, ClockPLL);
        ClockPLL = ClockPLL&0x3FF;
	pIM= (int)(pIM/(ClockPLL*1.065));
        return status;
    }

    byte VL53L1X_BootState(int dev, short state)
    {
        byte status = 0;
        short tmp = 0;

        status |= VL53L1_RdByte(dev,VL53L1_FIRMWARE__SYSTEM_STATUS, tmp);
	state = tmp;
        return status;
    }

    byte VL53L1X_GetSensorId(int dev, int sensorId)
    {
        byte status = 0;
        int tmp = 0;

        status |= VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, tmp);
	sensorId = tmp;
        return status;
    }

    byte VL53L1X_GetDistance(int dev, int distance)
    {
        byte status = 0;
        int tmp;

        status |= (VL53L1_RdWord(dev,
                VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, tmp));
	distance = tmp;
        return status;
    }

    byte VL53L1X_GetSignalPerSpad(int dev, int signalRate)
    {
        byte status = 0;
        int SpNb=1, signal;

        status |= VL53L1_RdWord(dev,
                VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, signal);
        status |= VL53L1_RdWord(dev,
                VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, SpNb);
	signalRate = (int) (200.0*signal/SpNb);
        return status;
    }

    byte VL53L1X_GetAmbientPerSpad(int dev, int *=ambPerSp)
    {
        byte status = 0;
        int AmbientRate, SpNb = 1;

        status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, AmbientRate);
        status |= VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, SpNb);
	ambPerSp=(int) (200.0 * AmbientRate / SpNb);
        return status;
    }

    byte VL53L1X_GetSignalRate(int dev, int signal)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,
                VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, tmp);
	signal = tmp*8;
        return status;
    }

    byte VL53L1X_GetSpadNb(int dev, int spNb)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,
                VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, tmp);
	spNb = tmp >> 8;
        return status;
    }

    byte VL53L1X_GetAmbientRate(int dev, int ambRate)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, tmp);
	ambRate = tmp*8;
        return status;
    }

    byte VL53L1X_GetRangeStatus(int dev, short rangeStatus)
    {
        byte status = 0;
        short RgSt;

	rangeStatus = 255;
        status |= VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, RgSt);
        RgSt = RgSt & 0x1F;
        if (RgSt < 24)
		rangeStatus = status_rtn[RgSt];
        return status;
    }

    byte VL53L1X_GetResult(int dev, VL53L1X_Result_t pResult)
    {
        byte status = 0;
        int [] Temp = new int[17];
        int RgSt = 255;

        status |= VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, Temp, 17);
        RgSt = Temp[0] & 0x1F;
        if (RgSt < 24)
            RgSt = status_rtn[RgSt];
        pResult.Status = RgSt;
        pResult.Ambient = (Temp[7] << 8 | Temp[8]) * 8;
        pResult.NumSPADs = Temp[3];
        pResult.SigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
        pResult.Distance = Temp[13] << 8 | Temp[14];

        return status;
    }

    byte VL53L1X_SetOffset(int dev, short OffsetValue)
    {
        byte status = 0;
        short Temp;

        Temp = (OffsetValue*4);
        status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
                (int)Temp);
        status |= VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
        status |= VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
        return status;
    }

    byte  VL53L1X_GetOffset(int dev, short offset)
    {
        byte status = 0;
        int Temp;

        status |= VL53L1_RdWord(dev,ALGO__PART_TO_PART_RANGE_OFFSET_MM, Temp);
        Temp = Temp<<3;
        Temp = Temp>>5;
	offset = (short)(Temp);
        return status;
    }

    byte VL53L1X_SetXtalk(int dev, int XtalkValue)
    {
        /* XTalkValue in count per second to avoid float type */
        byte status = 0;

        status |= VL53L1_WrWord(dev,
                ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
                0x0000);
        status |= VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
                0x0000);
        status |= VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
                (XtalkValue<<9)/1000); /* * << 9 (7.9 format) and /1000 to convert cps to kpcs */
        return status;
    }

    byte VL53L1X_GetXtalk(int dev, int xtalk )
    {
        byte status = 0;

        status |= VL53L1_RdWord(dev,ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalk);
	xtalk = (int)((xtalk*1000)>>9); /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
        return status;
    }

    byte VL53L1X_SetDistanceThreshold(int dev, int ThreshLow,
                                               int ThreshHigh, short Window,
                                               short IntOnNoTarget)
    {
        byte status = 0;
        short Temp = 0;

        status |= VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, Temp);
        Temp = Temp & 0x47;
        if (IntOnNoTarget == 0) {
            status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO,
                    (Temp | (Window & 0x07)));
        } else {
            status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO,
                    ((Temp | (Window & 0x07)) | 0x40));
        }
        status |= VL53L1_WrWord(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
        status |= VL53L1_WrWord(dev, SYSTEM__THRESH_LOW, ThreshLow);
        return status;
    }

    byte VL53L1X_GetDistanceThresholdWindow(int dev, int window)
    {
        byte status = 0;
        short tmp;
        status |= VL53L1_RdByte(dev,SYSTEM__INTERRUPT_CONFIG_GPIO, tmp);
	window = (int)(tmp & 0x7);
        return status;
    }

    byte VL53L1X_GetDistanceThresholdLow(int dev, int low)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,SYSTEM__THRESH_LOW, tmp);
	low = tmp;
        return status;
    }

    byte VL53L1X_GetDistanceThresholdHigh(int dev, int high)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,SYSTEM__THRESH_HIGH, tmp);
	high = tmp;
        return status;
    }

    byte VL53L1X_SetROICenter(int dev, short ROICenter)
    {
        byte status = 0;
        status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
        return status;
    }

    byte VL53L1X_GetROICenter(int dev, short ROICenter)
    {
        byte status = 0;
        short tmp;
        status |= VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, tmp);
	ROICenter = tmp;
        return status;
    }

    byte VL53L1X_SetROI(int dev, int X, int Y)
    {
        short OpticalCenter;
        byte status = 0;

        status |=VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, OpticalCenter);
        if (X > 16)
            X = 16;
        if (Y > 16)
            Y = 16;
        if (X > 10 || Y > 10){
            OpticalCenter = 199;
        }
        status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
        status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                (Y - 1) << 4 | (X - 1));
        return status;
    }

    byte VL53L1X_GetROI_XY(int dev, int ROI_X, int ROI_Y)
    {
        byte status = 0;
        short tmp;

        status = VL53L1_RdByte(dev,ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, tmp);
	ROI_X |= ((int)tmp & 0x0F) + 1;
	ROI_Y |= (((int)tmp & 0xF0) >> 4) + 1;
        return status;
    }

    byte VL53L1X_SetSignalThreshold(int dev, int Signal)
    {
        byte status = 0;

        status |= VL53L1_WrWord(dev,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,Signal>>3);
        return status;
    }

    byte VL53L1X_GetSignalThreshold(int dev, int signal)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,
                RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, tmp);
	signal = tmp <<3;
        return status;
    }

    byte VL53L1X_SetSigmaThreshold(int dev, int Sigma)
    {
        byte status = 0;

        if(Sigma>(0xFFFF>>2)){
            return 1;
        }
        /* 16 bits register 14.2 format */
        status |= VL53L1_WrWord(dev,RANGE_CONFIG__SIGMA_THRESH,Sigma<<2);
        return status;
    }

    byte VL53L1X_GetSigmaThreshold(int dev, int sigma)
    {
        byte status = 0;
        int tmp;

        status |= VL53L1_RdWord(dev,RANGE_CONFIG__SIGMA_THRESH, tmp);
	sigma = tmp >> 2;
        return status;

    }

    byte VL53L1X_StartTemperatureUpdate(int dev)
    {
        byte status = 0;
        short tmp=0;

        status |= VL53L1_WrByte(dev,VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x81); /* full VHV */
        status |= VL53L1_WrByte(dev,0x0B,0x92);
        status |= VL53L1X_StartRanging(dev);
        while(tmp==0){
            status |= VL53L1X_CheckForDataReady(dev, tmp);
        }
        tmp  = 0;
        status |= VL53L1X_ClearInterrupt(dev);
        status |= VL53L1X_StopRanging(dev);
        status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
        status |= VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
        return status;
    }
}
