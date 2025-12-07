# Code Example Verification

This directory contains verification scripts to ensure all code examples in the textbook are runnable in Docker and Colab environments.

## Verification Process

1. **Docker Verification**: All ROS 2 examples are tested in the Docker environment with ROS 2 Jazzy
2. **Colab Verification**: Python examples are tested in Google Colab-compatible environments
3. **Dependency Verification**: All required packages and dependencies are validated

## Test Results

| Module | Chapter | Code Example | Docker | Colab | Status |
|--------|---------|--------------|--------|-------|--------|
| Module 1 | Chapter 1 | ROS 2 Publisher | ✓ | ✗ (ROS 2 unavailable) | Working |
| Module 1 | Chapter 1 | ROS 2 Subscriber | ✓ | ✗ (ROS 2 unavailable) | Working |
| Module 1 | Chapter 3 | ROS 2 Service | ✓ | ✗ (ROS 2 unavailable) | Working |
| Module 1 | Chapter 4 | ROS 2 Action | ✓ | ✗ (ROS 2 unavailable) | Working |
| Module 2 | Chapter 6 | Gazebo Launch | ✓ | N/A | Working |
| Module 3 | Chapter 10 | Isaac ROS Pipeline | ✓ | N/A | Working |
| Module 4 | Chapter 13 | VLA Model | ✓ | ✓ | Working |
| Module 4 | Chapter 14 | Voice Processing | ✓ | ✓ | Working |

## Verification Scripts

- `test_docker.sh`: Runs all code examples in Docker containers
- `test_colab_notebooks.sh`: Verifies Colab compatibility
- `dependency_check.py`: Validates all required dependencies

## CI/CD Integration

All verification tests run automatically in GitHub Actions before deployment.