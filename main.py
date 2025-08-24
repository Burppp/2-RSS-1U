from ankle_ik import solve_motor_angles
import math

m1, m2 = solve_motor_angles(math.radians(10), math.radians(-5))
print(m1, m2)