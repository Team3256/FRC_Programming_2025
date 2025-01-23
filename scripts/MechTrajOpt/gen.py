import json
import math

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np


def main():
    N = 350

    ELEVATOR_START_HEIGHT = 3  # m
    ELEVATOR_END_HEIGHT = .7  # m
    ELEVATOR_MAX_VELOCITY = 1  # m/s
    ELEVATOR_MAX_ACCELERATION = 2.0  # m/s²

    ARM_LENGTH = 0.59055  # m
    ARM_START_ANGLE = -math.pi / 2  # rad
    ARM_END_ANGLE = 0  # rad
    ARM_MAX_VELOCITY = 2.0 * math.pi  # rad/s
    ARM_MAX_ACCELERATION = 4.0 * math.pi  # rad/s²

    END_EFFECTOR_MIN_HEIGHT = 0.44704 + 0.1  # m

    TOTAL_TIME = 7.0  # s

    problem = ca.Opti()

    dt = problem.variable(N)

    problem.subject_to(dt >= 0)
    problem.subject_to(dt <= TOTAL_TIME / N)
    for k in range(N - 1):
        problem.set_initial(dt[k], 0.05)
        problem.subject_to(dt[k] == dt[k + 1])

    elevator = problem.variable(2, N + 1)
    elevator_accel = problem.variable(1, N)

    arm = problem.variable(2, N + 1)
    arm_accel = problem.variable(1, N)

    for k in range(N):
        # Elevator dynamics constraints
        problem.subject_to(
            elevator[0, k + 1]
            == elevator[0, k]
            + elevator[1, k] * dt[k]
            + 0.5 * elevator_accel[0, k] * dt[k] ** 2
        )
        problem.subject_to(
            elevator[1, k + 1] == elevator[1, k] + elevator_accel[0, k] * dt[k]
        )

        # Arm dynamics constraints
        problem.subject_to(
            arm[0, k + 1]
            == arm[0, k] + arm[1, k] * dt[k] + 0.5 * arm_accel[0, k] * dt[k] ** 2
        )
        problem.subject_to(arm[1, k + 1] == arm[1, k] + arm_accel[0, k] * dt[k])

    # Elevator start and end conditions
    problem.subject_to(elevator[:, :1] == np.array([[ELEVATOR_START_HEIGHT], [0.0]]))
    problem.subject_to(
        elevator[:, N : N + 1] == np.array([[ELEVATOR_END_HEIGHT], [0.0]])
    )

    # Arm start and end conditions
    problem.subject_to(arm[:, :1] == np.array([[ARM_START_ANGLE], [0.0]]))
    problem.subject_to(arm[:, N : N + 1] == np.array([[ARM_END_ANGLE], [0.0]]))

    # Elevator velocity limits
    problem.subject_to(-ELEVATOR_MAX_VELOCITY <= elevator[1:2, :])
    problem.subject_to(elevator[1:2, :] <= ELEVATOR_MAX_VELOCITY)

    # Elevator acceleration limits
    problem.subject_to(-ELEVATOR_MAX_ACCELERATION <= elevator_accel)
    problem.subject_to(elevator_accel <= ELEVATOR_MAX_ACCELERATION)

    # Arm velocity limits
    problem.subject_to(-ARM_MAX_VELOCITY <= arm[1:2, :])
    problem.subject_to(arm[1:2, :] <= ARM_MAX_VELOCITY)

    # Arm acceleration limits
    problem.subject_to(-ARM_MAX_ACCELERATION <= arm_accel)
    problem.subject_to(arm_accel <= ARM_MAX_ACCELERATION)

    # Height limit
    problem.subject_to(
       elevator[:1, :] + ARM_LENGTH * np.sin(arm[:1, :])
       >= END_EFFECTOR_MIN_HEIGHT
    )

    # Cost function
    problem.minimize(sum([dt[i] for i in range(N)]))

    problem.solver("ipopt")
    sol = problem.solve()

    ts = [0]
    for i in range(N):
        ts.append(ts[-1] + sol.value(dt)[i])
    convert_to_json(ts, sol.value(arm))
    fig, axs = plt.subplots(3, 2)
    axs[0, 0].set_title("Arm Position (r)")
    axs[1, 0].set_title("Arm Velocity (r/s)")
    axs[2, 0].set_title("Arm Acceleration (r/s^2)")
    axs[0, 1].set_title("Elevator Position (m)")
    axs[1, 1].set_title("Elevator Velocity (m/s)")
    axs[2, 1].set_title("Elevator Acceleration (m/s^2)")
    axs[0, 0].plot(ts, sol.value(arm)[0, :].reshape(N + 1, 1), label="Position (m)")
    axs[1, 0].plot(ts, sol.value(arm)[1, :].reshape(N + 1, 1), label="Velocity (m/s)")
    axs[2, 0].plot(ts[:-1], sol.value(arm_accel).reshape(N, 1), label="Acceleration (m/s²)")
    axs[0, 1].plot(ts, sol.value(elevator[0, :]).reshape(N + 1, 1), label="Position (m)")
    axs[1, 1].plot(ts, sol.value(elevator[1, :]).reshape(N + 1, 1), label="Velocity (m/s)")
    axs[2, 1].plot(ts[:-1], sol.value(elevator_accel).reshape(N, 1), label="Acceleration (m/s²)")
    # plt.plot(ts, sol.value(arm[1, :]).reshape(N+1,1), label="Velocity (m/s)")
    # plt.plot(ts[:-1], sol.value(arm_accel).reshape(N, 1), label="Acceleration (m/s²)")
    plt.legend()
    plt.show()

def convert_to_json(ts, arm):
    data = {"data":[]}
    index = 0

    for i in range(len(ts)):
        if (ts[i] -ts[index]>=0.02) or i==0:
            data["data"].append({
                "time": ts[i],
                "position": arm[0, i],
             "velocity": arm[1, i]
            })
            index = i
    with open("data.json", "w") as f:
        json.dump(data, f, indent=3)
if __name__ == "__main__":
    main()