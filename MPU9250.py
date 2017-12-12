import time
import struct
from array import array
from OmegaExpansion import onionI2C

## MPU9250 Default I2C slave address
MPU9250_ADDRESS	= 0x68
AK8963_ADDRESS = 0x0C

DEVICE_ID		= 0x71

''' AK8963 Register Addresses '''
AK8963_ADDRESS   = 0x0C
AK8963_WHO_AM_I  = 0x00 #should return 0x48
AK8963_INFO      = 0x01
AK8963_ST1       = 0x02  # data ready status bit 0
AK8963_XOUT_L	 = 0x03  # data
AK8963_XOUT_H	 = 0x04
AK8963_YOUT_L	 = 0x05
AK8963_YOUT_H	 = 0x06
AK8963_ZOUT_L	 = 0x07
AK8963_ZOUT_H	 = 0x08
AK8963_ST2       = 0x09  # Data overflow bit 3 and data read error status bit 2
AK8963_CNTL      = 0x0A  # Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
AK8963_ASTC      = 0x0C  # Self test control
AK8963_I2CDIS    = 0x0F  # I2C disable
AK8963_ASAX      = 0x10  # Fuse ROM x-axis sensitivity adjustment value
AK8963_ASAY      = 0x11  # Fuse ROM y-axis sensitivity adjustment value
AK8963_ASAZ      = 0x12  # Fuse ROM z-axis sensitivity adjustment value

''' MPU-6500 Register Addresses '''
## sample rate driver

SELF_TEST_X_GYRO = 0x00                  
SELF_TEST_Y_GYRO = 0x01                                                                          
SELF_TEST_Z_GYRO = 0x02

SELF_TEST_X_ACCEL = 0x0D
SELF_TEST_Y_ACCEL = 0x0E    
SELF_TEST_Z_ACCEL = 0x0F

SELF_TEST_A = 0x10

XG_OFFSET_H = 0x13  # User-defined trim values for gyroscope
XG_OFFSET_L = 0x14
YG_OFFSET_H = 0x15
YG_OFFSET_L = 0x16
ZG_OFFSET_H = 0x17
ZG_OFFSET_L = 0x18


SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F

MOT_DUR = 0x20  # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
ZMOT_THR = 0x21  # Zero-motion detection threshold bits [7:0]
ZRMOT_DUR = 0x22  # Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

FIFO_EN        = 0x23
I2C_MST_CTRL   =  0x24   
I2C_SLV0_ADDR  =  0x25
I2C_SLV0_REG   =  0x26
I2C_SLV0_CTRL  =  0x27
I2C_SLV1_ADDR  =  0x28
I2C_SLV1_REG   =  0x29
I2C_SLV1_CTRL  =  0x2A
I2C_SLV2_ADDR  =  0x2B
I2C_SLV2_REG   =  0x2C
I2C_SLV2_CTRL  =  0x2D
I2C_SLV3_ADDR  =  0x2E
I2C_SLV3_REG   =  0x2F
I2C_SLV3_CTRL  =  0x30
I2C_SLV4_ADDR  =  0x31
I2C_SLV4_REG   =  0x32
I2C_SLV4_DO    =  0x33
I2C_SLV4_CTRL  =  0x34
I2C_SLV4_DI    =  0x35
I2C_MST_STATUS =  0x36
INT_PIN_CFG    =  0x37
INT_ENABLE     =  0x38
DMP_INT_STATUS =  0x39  # Check DMP interrupt
INT_STATUS     =  0x3A
ACCEL_XOUT_H   =  0x3B
ACCEL_XOUT_L   =  0x3C
ACCEL_YOUT_H   =  0x3D
ACCEL_YOUT_L   =  0x3E
ACCEL_ZOUT_H   =  0x3F
ACCEL_ZOUT_L   =  0x40
TEMP_OUT_H     =  0x41
TEMP_OUT_L     =  0x42
GYRO_XOUT_H    =  0x43
GYRO_XOUT_L    =  0x44
GYRO_YOUT_H    =  0x45
GYRO_YOUT_L    =  0x46
GYRO_ZOUT_H    =  0x47
GYRO_ZOUT_L    =  0x48
EXT_SENS_DATA_00 = 0x49
EXT_SENS_DATA_01 = 0x4A
EXT_SENS_DATA_02  = 0x4B
EXT_SENS_DATA_03  = 0x4C
EXT_SENS_DATA_04  = 0x4D
EXT_SENS_DATA_05  = 0x4E
EXT_SENS_DATA_06  = 0x4F
EXT_SENS_DATA_07  = 0x50
EXT_SENS_DATA_08  = 0x51
EXT_SENS_DATA_09  = 0x52
EXT_SENS_DATA_10  = 0x53
EXT_SENS_DATA_11  = 0x54
EXT_SENS_DATA_12  = 0x55
EXT_SENS_DATA_13  = 0x56
EXT_SENS_DATA_14  = 0x57
EXT_SENS_DATA_15  = 0x58
EXT_SENS_DATA_16  = 0x59
EXT_SENS_DATA_17  = 0x5A
EXT_SENS_DATA_18  = 0x5B
EXT_SENS_DATA_19  = 0x5C
EXT_SENS_DATA_20  = 0x5D
EXT_SENS_DATA_21  = 0x5E
EXT_SENS_DATA_22  = 0x5F
EXT_SENS_DATA_23  = 0x60
MOT_DETECT_STATUS  =  0x61
I2C_SLV0_DO       = 0x63
I2C_SLV1_DO     =  0x64
I2C_SLV2_DO     = 0x65
I2C_SLV3_DO     = 0x66
I2C_MST_DELAY_CTRL  = 0x67
SIGNAL_PATH_RESET  =  0x68
MOT_DETECT_CTRL  = 0x69
USER_CTRL    =    0x6A  # Bit 7 enable DMP, bit 3 reset DMP
PWR_MGMT_1   =    0x6B # Device defaults to the SLEEP mode
PWR_MGMT_2   =    0x6C
DMP_BANK     =    0x6D  # Activates a specific bank in the DMP
DMP_RW_PNT   =    0x6E  # Set read/write pointer to a specific start address in specified DMP bank
DMP_REG      =    0x6F  # Register in DMP from which to read or to which to write
DMP_REG_1    =    0x70
DMP_REG_2    =    0x71 
FIFO_COUNTH  =    0x72
FIFO_COUNTL  =    0x73
FIFO_R_W     =    0x74
WHO_AM_I_MPU9250  = 0x75 # Should return 0x71
XA_OFFSET_H   =   0x77
XA_OFFSET_L   =   0x78
YA_OFFSET_H   =   0x7A
YA_OFFSET_L   =   0x7B
ZA_OFFSET_H   =   0x7D
ZA_OFFSET_L   =   0x7E

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

GFS_250DPS = 0
GFS_500DPS = 1
GFS_1000DPS = 2
GFS_2000DPS = 3

AFS_2G = 0
AFS_4G = 1
AFS_8G = 2
AFS_16G = 3


i2c = onionI2C.OnionI2C()

class MPU9250:
	def __init__(self):
		self.gscale = GFS_1000DPS
		self.ascale = AFS_4G

		self.Self_test = array('f', [0,0,0,0,0,0])
		self.gx = 0
		self.gy = 0
		self.gz = 0

		self.ax = 0
		self.ay = 0
		self.az = 0

		self.g_bias = array('f', [0,0,0])
		self.a_bias = array('f', [0,0,0])

   	def init_MPU9250(self):
   		self.read_who_i_am()
   		# self.mpu_self_test_test()
   		self.mpu_self_test()
   		print self.Self_test

   		self.mpu_calibrate()
   		print self.g_bias[0]
   		print self.a_bias[0]
  #  		#wake up device
		# i2c.writeByte(MPU9250_ADDRESS,PWR_MGMT_1,0x00)
		# time.sleep(0.1)
		# #get stable time source
		# i2c.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
		# time.sleep(0.2)

		# #Configure Gyro
		# status = i2c.writeByte(MPU9250_ADDRESS, CONFIG, 0x03)

		# if status == 0:
		# 	print "no"
		# elif status == 1:
		# 	print "yes"
		# #Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		# i2c.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04)

  # 		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, self.gscale << 3)

  # 		i2c.writeByte(MPU9250_ADDRESS,ACCEL_CONFIG, self.ascale << 3)

  # 		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x03) # Write new ACCEL_CONFIG2 register value

  # 		#Configure Interrupts and Bypass Enable
  # 		i2c.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02)
  #  		#i2c.writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01) # Enable data ready (bit 0) interrupt
  #  		time.sleep(0.1);


   	def read_who_i_am(self):
   		c = i2c.readBytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250,1)
   		hex_c = [hex(data) for data in c]
   		print "MPU9250 , I AM {} , I should be 0x71.".format(hex_c)


	def read_accel(self):
		accel_list = i2c.readBytes(MPU9250_ADDRESS,ACCEL_OUT,6)
		accel_X = (accel_list[0] << 8) | accel_list[1]
		accel_Y = (accel_list[2] << 8) | accel_list[3]
		accel_Z = (accel_list[4] << 8) | accel_list[5]

		return accel_X,accel_Y,accel_Z

	def read_gyro(self):
		gyro_list = i2c.readBytes(MPU9250_ADDRESS,GYRO_OUT,6)
		gyro_X = (gyro_list[0] << 8) | gyro_list[1]
		gyro_Y = (gyro_list[2] << 8) | gyro_list[3]
		gyro_Z = (gyro_list[4] << 8) | gyro_list[5]

		return gyro_X,gyro_Y,gyro_Z
	def mpu_self_test_test(self):

		raw_input = [0,0,0,0,0,0]

		FS = 0x00 # uint8_t
		i2c.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00) #Set gyro sample rate to 1kHz
		i2c.writeByte(MPU9250_ADDRESS, CONFIG, 0x02) #Set gyro sample rate to 1kHz and DLPF to 92Hz
		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<3) #Set full scale range for the gyro to 250 dps
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x02); # Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); # Set full scale range for the accelerometer to 2 g

		raw_input = i2c.readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)

		rc0 = chr(raw_input[0])
		rc1 = chr(raw_input[1])
		result = struct.unpack('>h',rc0+rc1)[0]
		print result 
		# for data in raw_input:
		# 	print bin(data)


	def mpu_self_test(self):

		raw_data = []
		self_test = array('f', [0,0,0,0,0,0])
		aAvg, gAvg, aSTAvg, gSTAvg = [0,0,0],[0,0,0],[0,0,0],[0,0,0]
		# raw_data = array('b',[0,0,0,0,0,0])
		# self_test = array('b',[0,0,0,0,0,0])
		# aAvg, gAvg, aSTAvg, gSTAvg = array('l',[0,0,0]), array('l',[0,0,0]), array('l',[0,0,0]), array('l',[0,0,0])
		factoryTrim = array('f', [0,0,0,0,0,0])
		FS = 0x00 # uint8_t

		i2c.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00) #Set gyro sample rate to 1kHz
		i2c.writeByte(MPU9250_ADDRESS, CONFIG, 0x02) #Set gyro sample rate to 1kHz and DLPF to 92Hz
		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<3) #Set full scale range for the gyro to 250 dps
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x02); # Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); # Set full scale range for the accelerometer to 2 g

		for i in xrange(200):
			# raw_data[0],raw_data[1] = i2c.readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 2)
			# raw_data[2],raw_data[3] = i2c.readBytes(MPU9250_ADDRESS, ACCEL_YOUT_H, 2)
			# raw_data[4],raw_data[5] = i2c.readBytes(MPU9250_ADDRESS, ACCEL_ZOUT_H, 2)
			print i2c.readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)
			raw_data = i2c.readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)

			#print raw_data
			aAvg[0] += struct.unpack('>h',chr(raw_data[0])+chr(raw_data[1]))[0]
			aAvg[1] += struct.unpack('>h',chr(raw_data[2])+chr(raw_data[3]))[0]
			aAvg[2] += struct.unpack('>h',chr(raw_data[4])+chr(raw_data[5]))[0]

			raw_data = i2c.readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6)
			gAvg[0] += struct.unpack('>h',chr(raw_data[0])+chr(raw_data[1]))[0]
			gAvg[1] += struct.unpack('>h',chr(raw_data[2])+chr(raw_data[3]))[0]
			gAvg[2] += struct.unpack('>h',chr(raw_data[4])+chr(raw_data[5]))[0]

		for i in xrange(3):
			aAvg[i] /= 200
			gAvg[i] /= 200

		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0) # Enable self test on all three axes and set accelerometer range to +/- 2 g
   		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0) # Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   		time.sleep(25/1000) # 25ms

		for i in xrange(200):
			raw_data = i2c.readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)
			aSTAvg[0] += struct.unpack('>h',chr(raw_data[0])+chr(raw_data[1]))[0]
			aSTAvg[1] += struct.unpack('>h',chr(raw_data[2])+chr(raw_data[3]))[0]
			aSTAvg[2] += struct.unpack('>h',chr(raw_data[4])+chr(raw_data[5]))[0]

			raw_data = i2c.readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6)
			gSTAvg[0] += struct.unpack('>h',chr(raw_data[0])+chr(raw_data[1]))[0]
			gSTAvg[1] += struct.unpack('>h',chr(raw_data[2])+chr(raw_data[3]))[0]
			gSTAvg[2] += struct.unpack('>h',chr(raw_data[4])+chr(raw_data[5]))[0]

		for i in xrange(3):
			aSTAvg[i] /= 200
			gSTAvg[i] /= 200


		# Configure the gyro and accelerometer for normal operation
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00)  
		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00)
		time.sleep(25/1000) # 25ms

		# Retrieve accelerometer and gyro factory Self-TesSELF_TEST_X_GYROt Code from USR_Reg
		self_test[0] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_X_ACCEL , 1)[0]
		self_test[1] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL , 1)[0]
		self_test[2] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL , 1)[0]
		self_test[3] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_X_GYRO , 1)[0]
		self_test[4] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_Y_GYRO , 1)[0]
		self_test[5] = i2c.readBytes(MPU9250_ADDRESS, SELF_TEST_Z_GYRO , 1)[0]

		print "-----"
		# Retrieve factory self-test value from self-test code reads
		for i in xrange(6):
			factoryTrim[i] = (2620/1<<FS)*(pow( 1.01 , (self_test[i] - 1.0) ))
		#Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
		#To get percent, must multiply by 100
		for i in xrange(3):
			print "a : {0} , {1} , {2}".format(aSTAvg[i],aAvg[i],factoryTrim[i])
			self.Self_test[i] = 100.0*(aSTAvg[i] - aAvg[i])/factoryTrim[i] - 100
			print "g : {0} , {1} , {2}".format(gSTAvg[i],gAvg[i],factoryTrim[i+3])
			self.Self_test[i+3] = 100.0*(gSTAvg[i] - gAvg[i])/factoryTrim[i+3] - 100

	def mpu_calibrate(self):

		i2c.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80) # Write a one to bit 7 reset bit; toggle reset device
		time.sleep(100/1000)

		# get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
		# else use the internal oscillator, bits 2:0 = 001
		i2c.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01)  
		i2c.writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00)
		time.sleep(200/1000)

		# Configure device for bias calculation
		i2c.writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00)   # Disable all interrupts
		i2c.writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00)    # Disable FIFO
		i2c.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)  # Turn on internal clock source
		i2c.writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00) # Disable I2C master
		i2c.writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00)  # Disable FIFO and I2C master modes
		i2c.writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C)  # Reset FIFO and DMP
		time.sleep(15/1000)

		i2c.writeByte(MPU9250_ADDRESS, CONFIG, 0x01)      # Set low-pass filter to 188 Hz
		i2c.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00)  # Set sample rate to 1 kHz
		i2c.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00)  # Set gyro full-scale to 250 degrees per second, maximum sensitivity
		i2c.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00) #  Set accelerometer full-scale to 2 g, maximum sensitivity

		gyrosensitivity = 131 # = 131 LSB/degrees/sec
		accelsensitivity = 16384 # = 16384 LSB/g

		i2c.writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40) #Enable FIFO
		i2c.writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78) #Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		time.sleep(40/1000) #accumulate 40 samples in 40 milliseconds = 480 bytes

		i2c.writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00)
		fifo_count_data = i2c.readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2)
		fifo_count = struct.unpack('>h',chr(fifo_count_data[0])+chr(fifo_count_data[1]))[0]
		packet_count = fifo_count/12

		data = [0,0,0,0,0,0,0,0,0,0,0,0]
		accel_bias_temp = [0,0,0]
		gyro_bias_temp = [0,0,0]

		for i in xrange(packet_count):
			accel_temp, gyro_temp = [0,0,0], [0,0,0]
			data = i2c.readBytes(MPU9250_ADDRESS, FIFO_R_W, 12)
			accel_bias_temp[0] += struct.unpack('>h',chr(data[0])+chr(data[1]))[0] # Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
			accel_bias_temp[1] += struct.unpack('>h',chr(data[2])+chr(data[3]))[0]
			accel_bias_temp[2] += struct.unpack('>h',chr(data[4])+chr(data[5]))[0]
			gyro_bias_temp[0] += struct.unpack('>h',chr(data[0])+chr(data[1]))[0]
			gyro_bias_temp[1] += struct.unpack('>h',chr(data[2])+chr(data[3]))[0]
			gyro_bias_temp[2] += struct.unpack('>h',chr(data[4])+chr(data[5]))[0]

		for i in xrange(3):
			print accel_bias_temp[i]
		for i in xrange(3):
			print gyro_bias_temp[i]


		accel_bias_temp[0] = accel_bias_temp[0]/packet_count # Normalize sums to get average count biases
		accel_bias_temp[1] = accel_bias_temp[1]/packet_count
		accel_bias_temp[2] = accel_bias_temp[2]/packet_count
		gyro_bias_temp[0] = gyro_bias_temp[0]/packet_count
		gyro_bias_temp[1] = gyro_bias_temp[1]/packet_count
		gyro_bias_temp[2] = gyro_bias_temp[2]/packet_count

		if accel_bias_temp[2] > 0L : # Remove gravity from the z-axis accelerometer bias calculation
			accel_bias_temp[2] -= accelsensitivity
		else:
			accel_bias_temp[2] += accelsensitivity

		# --- Gyro calibratation
		for i in xrange(3):
			data[i] = (-gyro_bias_temp[i] / 4 >> 8 ) & 0xFF # Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
			data[i+1] = (-gyro_bias_temp[i] / 4 ) & 0xFF    # Biases are additive, so change sign on calculated average gyro biases

		for i in xrange(6):
			print bin(data[i])

		i2c.writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0])
		i2c.writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1])
		i2c.writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2])
		i2c.writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3])
		i2c.writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4])
		i2c.writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5])

		for i in xrange(3):
			self.g_bias[i] = float(gyro_bias_temp[i])/float(gyrosensitivity)
		# --- Accelerometer calibratation

		accel_bias_reg = [0,0,0]
		data = i2c.readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 6)
		accel_bias_reg[0] = struct.unpack('>h',chr(data[0])+chr(data[1]))[0]
		accel_bias_reg[1] = struct.unpack('>h',chr(data[2])+chr(data[3]))[0]
		accel_bias_reg[2] = struct.unpack('>h',chr(data[4])+chr(data[5]))[0]



		mask = 1L
		mask_bit = [0,0,0]

		for i in xrange(3):
			if accel_bias_reg[i] & mask:
				mask_bit[i] = 0x01

		accel_bias_reg[0] -= accel_bias_temp[0]/8
		accel_bias_reg[1] -= accel_bias_temp[1]/8
		accel_bias_reg[2] -= accel_bias_temp[2]/8

		data[0] = (accel_bias_reg[0] >> 8) & 0xFF
		data[1] = (accel_bias_reg[0])
		data[1] = data[1] | mask_bit[0]
		data[2] = (accel_bias_reg[1] >> 8) & 0xFF
		data[3] = (accel_bias_reg[1])
		data[3] = data[3] | mask_bit[1]
		data[4] = (accel_bias_reg[2] >> 8) & 0xFF
		data[5] = (accel_bias_reg[2])
		data[5] = data[5] | mask_bit[2]

		for i in xrange(6):
			print bin(data[i])

		i2c.writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0])
		i2c.writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1])
		i2c.writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2])
		i2c.writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3])
		i2c.writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4])
		i2c.writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5])

		self.a_bias[0] = float(accel_bias_temp[0]) / float(accelsensitivity)
		self.a_bias[1] = float(accel_bias_temp[1]) / float(accelsensitivity)
		self.a_bias[2] = float(accel_bias_temp[2]) / float(accelsensitivity)


















































