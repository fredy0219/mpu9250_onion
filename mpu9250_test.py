from MPU9250 import MPU9250
import time
mpu9250 = MPU9250()

if __name__ == '__main__':
	mpu9250.init_MPU9250()
	# mpu6500.read_who_i_am()
	# while True:
	# 	ax,ay,az = mpu6500.read_accel()
	# 	print "{} , {} , {}".format(ax,ay,az)

	# 	gx,gy,gz = mpu6500.read_gyro()
	# 	print "{} , {} , {}".format(gx,gy,gz)

	# 	time.sleep(1)

	for i in xrange(10):
		axt,ayt,azt = mpu9250.read_accel()
		mpu9250.get_ares()

		ax = axt * mpu9250.ares
		ay = ayt * mpu9250.ares
		az = azt * mpu9250.ares

		print "X-acceleration: {0} mg".format(1000*ax)
		print "Y-acceleration: {0} mg".format(1000*ay)
		print "Z-acceleration: {0} mg".format(1000*az)


