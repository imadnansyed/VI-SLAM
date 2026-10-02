from pre_integration_module import PreIntegrator
import numpy as np

"""
Frame 0
   ↓
integrate IMU samples
   ↓
get_measurement()
   ↓
save ΔR_01, Δv_01, Δp_01, Σ_01
   ↓
reset()
   ↓
Frame 1
   ↓
integrate IMU samples
   ↓
get_measurement()
   ↓
save ΔR_12, Δv_12, Δp_12, Σ_12
"""

"""
Sample Data :
gyro = [0, 0, 0] rad/s
accel = [1, 0, 0] m/s²
dt = 0.01 s
"""

preintegrator = PreIntegrator(
            gyro_noise_cov=0.0001 * np.eye(3),
            accel_noise_cov=0.001 * np.eye(3),

            ) 

dt = 0.01
gyro = np.array([0.0, 0.0, 0.0])
accel = np.array([1.0, 0.0, 0.0])

# FRAME 0 -> FRAME 1

for _ in range(10):
    preintegrator.integrate(gyro, accel, dt)

measurement_01 = preintegrator.get_measurement()
covariance_01 = preintegrator.get_covariance()
jacobians_01 = preintegrator.get_bias_jacobians()

print("\nFRAME 0 -> FRAME 1")
print("Delta R:")
print(measurement_01["delta_R"])

print("Delta v:")
print(measurement_01["delta_v"])

print("Delta p:")
print(measurement_01["delta_p"])

print("Delta t:")
print(measurement_01["delta_t"])

print("Covariance trace:")
print(np.trace(covariance_01))


# RESET

preintegrator.reset()


# FRAME 1 -> FRAME 2

for _ in range(10):
    preintegrator.integrate(gyro, accel, dt)

measurement_12 = preintegrator.get_measurement()
covariance_12 = preintegrator.get_covariance()
jacobians_12 = preintegrator.get_bias_jacobians()

print("\nFRAME 1 -> FRAME 2")
print("Delta R:")
print(measurement_12["delta_R"])

print("Delta v:")
print(measurement_12["delta_v"])

print("Delta p:")
print(measurement_12["delta_p"])

print("Delta t:")
print(measurement_12["delta_t"])

print("Covariance trace:")
print(np.trace(covariance_12))