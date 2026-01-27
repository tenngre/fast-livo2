#!/usr/bin/env python3
import os
import sys
import argparse
import subprocess


def evaluate_trajectory(trajectory_file, groundtruth_file=None, output_dir="."):
    """
    Evaluate trajectory using evo library
    
    Args:
        trajectory_file: Path to the estimated trajectory file (TUM format)
        groundtruth_file: Path to the ground truth trajectory file (TUM format)
        output_dir: Directory to save evaluation results
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Evaluating trajectory: {trajectory_file}")
    
    # Step 1: Plot the trajectory
    print("\n=== Plotting trajectory ===")
    traj_cmd = ["evo_traj", "tum", trajectory_file, "-p", "--plot_mode", "xyz"]
    if groundtruth_file:
        traj_cmd.extend(["--ref", groundtruth_file])
    subprocess.run(traj_cmd)
    
    # Step 2: Compute Absolute Pose Error (APE) if ground truth is provided
    if groundtruth_file:
        print("\n=== Computing Absolute Pose Error (APE) ===")
        ape_cmd = ["evo_ape", "tum", groundtruth_file, trajectory_file, "-va", "--plot", "--plot_mode", "xyz", 
                   "--save_results", os.path.join(output_dir, "ape_results.zip")]
        subprocess.run(ape_cmd)
        
        # Step 3: Compute Relative Pose Error (RPE) if ground truth is provided
        print("\n=== Computing Relative Pose Error (RPE) ===")
        rpe_cmd = ["evo_rpe", "tum", groundtruth_file, trajectory_file, "-va", "--plot", "--plot_mode", "xyz", 
                   "--save_results", os.path.join(output_dir, "rpe_results.zip")]
        subprocess.run(rpe_cmd)
    else:
        print("\nNo ground truth provided. Skipping APE and RPE computation.")
    
    print(f"\nEvaluation completed. Results saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate FAST-LIVO2 trajectory using evo library")
    parser.add_argument("--trajectory", type=str, required=True, help="Path to estimated trajectory file (TUM format)")
    parser.add_argument("--groundtruth", type=str, default=None, help="Path to ground truth trajectory file (TUM format)")
    parser.add_argument("--output", type=str, default="./evaluation_results", help="Directory to save evaluation results")
    
    args = parser.parse_args()
    
    evaluate_trajectory(args.trajectory, args.groundtruth, args.output)


if __name__ == "__main__":
    main()
