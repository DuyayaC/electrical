#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"



Class_PID IMU_PID;

fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;
BMI088 BMI088;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void Calibrate_BMI088(BMI088 *BMI088_offset);

//校准零漂
void Calibrate_BMI088(Calibrate *BMI088_offset)
{
    static uint16_t CaliTimes = 6000; //获取足够多的数据点
	  static uint16_t acc_CaliTimes = 6000; // 获取足够多的数据点
		static float startTime;
		uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
		int16_t bmi088_raw_temp;
		float gyroMax[3], gyroMin[3];
		float gNormTemp, gNormMax, gNormMin;
		
		
		startTime = DWT_GetTimeline_s();
		do
		{
			if (DWT_GetTimeline_s() - startTime > 10)
			{
				//校准超时
				BMI088_offset->GyroOffset[0] = GxOFFSET;
				BMI088_offset->GyroOffset[0] = GxOFFSET;
				BMI088_offset->GyroOffset[0] = GxOFFSET;
				BMI088_offset->gNorm = gNORM;
				break;
			}
			
			DWT_Delay(0.005);
			BMI088_offset->gNorm = 0;
			BMI088_offset->GyroOffset[0] = 0;
			BMI088_offset->GyroOffset[1] = 0;
			BMI088_offset->GyroOffset[2] = 0;
			
			for (uint16_t counter = 0; counter < CaliTimes; counter++)
			{
				//三轴加速度
				BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
				bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
				BMI088_offset->accel_temp[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
				bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
				BMI088_offset->accel_temp[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
				bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
				BMI088_offset->accel_temp[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
				gNormTemp = sqrtf(BMI088_offset->accel_temp[0] * BMI088_offset->accel_temp[0] +
													BMI088_offset->accel_temp[1] * BMI088_offset->accel_temp[1] +
													BMI088_offset->accel_temp[2] * BMI088_offset->accel_temp[2]);
				BMI088_offset->gNorm += gNormTemp;
				
				//三轴角速度
				BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
				if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
				{
						 bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
						 BMI088_offset->gyro_temp[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
						 BMI088_offset->GyroOffset[0] += BMI088_offset->gyro_temp[0];
						 bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
						 BMI088_offset->gyro_temp[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
						 BMI088_offset->GyroOffset[1] += BMI088_offset->gyro_temp[1];
						 bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
						 BMI088_offset->gyro_temp[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
						 BMI088_offset->GyroOffset[2] += BMI088_offset->gyro_temp[2];
				}
				
				//记录数据极差
				if (counter == 0)
				{
					gNormMax = gNormTemp;
					gNormMin = gNormTemp;
					for (uint8_t j = 0; j < 3; j++)
					{
						gyroMax[j] = BMI088_offset->gyro_temp[j];
						gyroMin[j] = BMI088_offset->gyro_temp[j];
					}
				}
				else
				{
					//更新最值
					if (gNormTemp > gNormMax)
							gNormMax = gNormTemp;
					if (gNormTemp < gNormMax)
							gNormMin = gNormTemp;
					for (uint8_t j = 0; j < 3; j++)
					{
						if (BMI088_offset->gyro_temp[j] > gyroMax[j])
								gyroMax[j] = BMI088_offset->gyro_temp[j];
						if (BMI088_offset->gyro_temp[j] < gyroMin[j])
								gyroMin[j] = BMI088_offset->gyro_temp[j];
					}
				}
				
				//数据差异过大认为收到干扰，重新校准
				gNormDiff = gNormMax - gNormMin;
				for (uint8_t j = 0; j < 3; j++)
						gyroDiff[j] = gyroMax[j] - gyroMin[j];
				if (gNormDiff > 0.5f ||
						gyroDiff[0] > 0.15f ||
						gyroDiff[1] > 0.15f ||
						gyroDiff[2] > 0.15f)
						break;
				DWT_Delay(0.0005);
			}
				//取平均值得到计算结果
				BMI088_offset->gNorm /= (float)CaliTimes;
				for (uint8_t i = 0; i < 3; i++)
						BMI088_offset->GyroOffset[i] /= (float)CaliTimes;
				BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
				bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
				if (bmi088_raw_temp > 1023)
						bmi088_raw_temp -= 2048;
				BMI088_offset->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
			
				caliCount++;
		}while (gNormDiff > 0.5f ||
						fabsf(BMI088_offset->gNorm - 9.8f) > 0.5f ||
						gyroDiff[0] > 0.15f ||
						gyroDiff[1] > 0.15f ||
						gyroDiff[2] > 0.15f ||
						fabsf(BMI088_offset->GyroOffset[0]) > 0.01f ||
						fabsf(BMI088_offset->GyroOffset[0]) > 0.01f ||
						fabsf(BMI088_offset->GyroOffset[0]) > 0.01f);
		BMI088_offset->AccelScale = 9.81f / BMI088_offset->gNorm;
		for (uint16_t i = 0; i < acc_CaliTimes; i++)
     {
		     
      BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

      bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
      BMI088_offset->accel_temp[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088_offset->AccelScale;
			BMI088_offset->AccelOffset[0] = BMI088_offset->AccelOffset[0] * 0.8f + BMI088_offset->accel_temp[0] * 0.2f;//加速度一阶低通滤波
			 
      bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_offset->accel_temp[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088_offset->AccelScale;
			BMI088_offset->AccelOffset[1] = BMI088_offset->AccelOffset[1] * 0.8f + BMI088_offset->accel_temp[1] * 0.2f;//加速度一阶低通滤波
			 
      bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_offset->accel_temp[2]= bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088_offset->AccelScale;
			BMI088_offset->AccelOffset[2] = BMI088_offset->AccelOffset[2] * 0.8f + BMI088_offset->accel_temp[2] * 0.2f;//加速度一阶低通滤波
		  	 
			DWT_Delay(0.0005);		 
		 }
}


uint8_t BMI088_init()
{
	uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();
    IMU_PID.Init(2000.0f, 200.0f, 0.2f, 0.0f, 4500.0f, 4500.0f);

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    
    return error;
}

bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}





void BMI088_read(BMI088 *BMI088)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
		//读取三轴加速度
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
		
	bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
	BMI088->[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088->AccelScale; //- BMI088_offset->AccelOffset[0];

	bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
	BMI088->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088->AccelScale; //- BMI088_offset->AccelOffset[1];
		
	bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
	BMI088->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * BMI088->AccelScale; //- BMI088_offset->AccelOffset[2];
		
		
		//读取三轴角速度
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
		bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
		BMI088->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - BMI088->GyroOffset[0];
			
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        BMI088->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - BMI088->GyroOffset[1];
			
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        BMI088->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - BMI088->GyroOffset[2];
			
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    BMI088->temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}


static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

void IMU_Temperature_Control(float temp)
{
		uint16_t PWM;
    float outcheck;
    IMU_PID.Set_Target(40.0f);
    IMU_PID.Set_Now(temp);
    IMU_PID.TIM_Adjust_PeriodElapsedCallback();
    outcheck = IMU_PID.Get_Out();
    if (outcheck < 0.0f)
			{
        outcheck = 0.0f;
			}
    PWM = (uint16_t)outcheck;
    TIM10->CCR1 = (PWM);		
}

void IMU_Calibrate(uint8_t calibrate)
{
	  if (calibrate)
		{
			Calibrate_BMI088(&BMI088);
		}
        
    else
    {
        BMI088_offset.GyroOffset[0] = GxOFFSET;
        BMI088_offset.GyroOffset[1] = GyOFFSET;
        BMI088_offset.GyroOffset[2] = GzOFFSET;
        BMI088_offset.gNorm = gNORM;
        BMI088_offset.AccelScale = 9.81f / BMI088_offset.gNorm;
        BMI088_offset.TempWhenCali = 40;
    }
		return;
}
#elif defined(BMI088_USE_IIC)

#endif
