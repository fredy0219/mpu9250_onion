from MPU9250 import MPU9250
import time
mpu6500 = MPU9250()

if __name__ == '__main__':
	mpu6500.init_MPU9250()
	# mpu6500.read_who_i_am()
	# while True:
	# 	ax,ay,az = mpu6500.read_accel()
	# 	print "{} , {} , {}".format(ax,ay,az)

	# 	gx,gy,gz = mpu6500.read_gyro()
	# 	print "{} , {} , {}".format(gx,gy,gz)

	# 	time.sleep(1)
	

