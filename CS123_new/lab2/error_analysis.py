import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics(joint_pos, theta1, theta2, theta3, error=None):

    def rotation_x(angle):
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def translation(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    def add_error(x, y, z, error):
        # Apply error such that the Euclidean distance is exactly `error`
        vector = np.random.randn(3)  # random unit vector direction
        vector /= np.linalg.norm(vector)  # normalize to unit length
        vector *= error  # scale to the exact error distance

        return translation(x + vector[0], y + vector[1], z + vector[2])

    # T_0_1 (base_link to leg_front_r_1)
    T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

    # T_1_2 (leg_front_r_1 to leg_front_r_2)
    if error is not None:
        T_1_2 = add_error(0, 0, 0.039, error) @ rotation_y(-np.pi / 2) @ rotation_z(theta2)
    else:
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-np.pi / 2) @ rotation_z(theta2)

    # T_2_3 (leg_front_r_2 to leg_front_r_3)
    T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(theta3)

    # T_3_ee (leg_front_r_3 to end-effector)
    T_3_ee = translation(0.06231, -0.06216, 0.018)

    # Final transformation
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

    # Extract the end-effector position
    end_effector_position = T_0_ee @ joint_pos
    return end_effector_position[:3]  # Return only the x, y, z coordinates

def test_error_impact(errors, num_tests=10):
    joint_pos = np.array([0, 0, 0, 1])  # Homogeneous joint position
    results = {error: [] for error in errors}

    for _ in range(num_tests):
        theta1 = np.random.uniform(0, 2 * np.pi)
        theta2 = np.random.uniform(0, 2 * np.pi)
        theta3 = np.random.uniform(0, 2 * np.pi)

        for error in errors:
            end_effector_pos = forward_kinematics(joint_pos, theta1, theta2, theta3, error if error > 0 else None)
            results[error].append(end_effector_pos)
    for error in errors:
        results[error] = np.array(results[error])
    # Calculate average end-effector positions for each error level
    avg_results = {error: np.sqrt(np.mean(np.sum((positions - results[0])**2, axis=1), axis=0)) for error, positions in results.items()}

    # Display the results
    for error, avg_pos in avg_results.items():
        print(f"Error: {error} - Average End-Effector Offset: {avg_pos}")

    return results

def plot_end_effector_positions(errors, results):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot ground truth (no error)
    ground_truth = np.array(results[0])
    ax.scatter(ground_truth[:, 0], ground_truth[:, 1], ground_truth[:, 2], label='Ground Truth', color='blue')

    # Plot points for different error values
    colors = ['red', 'green', 'orange']
    for i, error in enumerate(errors[1:], start=1):
        errored_points = np.array(results[error])
        ax.scatter(errored_points[:, 0], errored_points[:, 1], errored_points[:, 2], label=f'Error {error}', color=colors[i-1])

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.legend()
    plt.show()

# Running the tests with error values of 0 (ground truth), 0.002, 0.004, and 0.008
errors = [0, 0.002, 0.004, 0.008]
results = test_error_impact(errors, num_tests=100)

# Plot the results
plot_end_effector_positions(errors, results)
