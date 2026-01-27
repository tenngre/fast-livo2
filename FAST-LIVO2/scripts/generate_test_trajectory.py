#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt


def generate_test_trajectory(output_file, num_points=100, noise_level=0.01):
    """
    Generate a test trajectory in TUM format
    
    Args:
        output_file: Path to save the trajectory file
        num_points: Number of points in the trajectory
        noise_level: Standard deviation of Gaussian noise to add
    """
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Generate time stamps
    times = np.linspace(0, 10, num_points)
    
    # Generate a circular trajectory
    radius = 5.0
    x = radius * np.cos(times)
    y = radius * np.sin(times)
    z = 0.1 * times  # Slight upward trend
    
    # Add Gaussian noise
    x += np.random.normal(0, noise_level, num_points)
    y += np.random.normal(0, noise_level, num_points)
    z += np.random.normal(0, noise_level, num_points)
    
    # Generate quaternions (identity rotation)
    qx = np.zeros(num_points)
    qy = np.zeros(num_points)
    qz = np.zeros(num_points)
    qw = np.ones(num_points)
    
    # Write to TUM format file
    with open(output_file, 'w') as f:
        for i in range(num_points):
            f.write(f"{times[i]:.9f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} {qx[i]:.6f} {qy[i]:.6f} {qz[i]:.6f} {qw[i]:.6f}\n")
    
    print(f"Generated test trajectory with {num_points} points: {output_file}")
    
    # Plot the trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, '-o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Test Trajectory')
    plt.savefig(os.path.join(os.path.dirname(output_file), 'test_trajectory.png'))
    print(f"Saved trajectory plot: {os.path.join(os.path.dirname(output_file), 'test_trajectory.png')}")


def main():
    output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Log', 'result')
    output_file = os.path.join(output_dir, 'test_trajectory.txt')
    generate_test_trajectory(output_file)


if __name__ == "__main__":
    main()
