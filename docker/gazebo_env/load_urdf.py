#!/usr/bin/env python3

import os
import sys
import argparse
from pathlib import Path


def load_urdf_file(urdf_path):
    """
    Load a URDF file and return its content.

    Args:
        urdf_path (str): Path to the URDF file

    Returns:
        str: Content of the URDF file
    """
    try:
        with open(urdf_path, 'r') as file:
            urdf_content = file.read()
        return urdf_content
    except FileNotFoundError:
        print(f"Error: URDF file not found at {urdf_path}")
        return None
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description='Load and display URDF file for Gazebo simulation')
    parser.add_argument('urdf_path', type=str, help='Path to the URDF file')
    parser.add_argument('--validate', action='store_true', help='Validate URDF syntax')

    args = parser.parse_args()

    # Check if the URDF file exists
    urdf_path = Path(args.urdf_path)
    if not urdf_path.exists():
        print(f"Error: URDF file does not exist: {args.urdf_path}")
        sys.exit(1)

    # Load the URDF file
    urdf_content = load_urdf_file(urdf_path)
    if urdf_content is None:
        sys.exit(1)

    print(f"Successfully loaded URDF file: {urdf_path}")

    if args.validate:
        # Simple validation - check if it's a valid XML with robot tag
        if '<robot' in urdf_content and '</robot>' in urdf_content:
            print("URDF validation passed: Basic XML structure is valid")
        else:
            print("URDF validation failed: Missing robot tags")
            sys.exit(1)

    # Print first 1000 characters of URDF for verification
    print("\nFirst 1000 characters of URDF:")
    print(urdf_content[:1000])
    if len(urdf_content) > 1000:
        print("...")

    return urdf_content


if __name__ == "__main__":
    main()