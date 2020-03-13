# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "./main.c ./i2c_comm/i2c_comm.c ./imu_GY86/mpu6050/mpu.c ./imu_GY86/hmc5883/hmc.c ./imu_GY86/ms5611/ms.c ./rc_receiver/rc_receiver.c ./bldc/bldc.c ./microtimer/microtimer.c ./geometry/geometry.c ./sensor_loop.c ./state.c ./pid/pid.c ./pid/pid_manager.c ./bldc/motor_manager.c")
set(COMPONENT_ADD_INCLUDEDIRS "./imu_GY86 ./rc_receiver ./bldc ./i2c_comm ./microtimer ./geometry ./pid .")

register_component()
