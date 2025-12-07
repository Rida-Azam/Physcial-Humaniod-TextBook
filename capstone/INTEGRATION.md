# Integration of Voice Processing, LLM Planning, and ROS Execution

This document describes the integration of voice processing (Whisper), LLM planning, and ROS execution for the humanoid robotics capstone project.

## Architecture Overview

The system integrates multiple components to create an end-to-end pipeline from spoken commands to robot actions:

```
[Voice Input] → [Whisper] → [NLU] → [LLM Planner] → [ROS Executor] → [Robot Actions]
```

## Component Descriptions

### 1. Voice Processing (Whisper)
- **Node**: `voice_processor.py`
- **Function**: Converts speech to text using OpenAI's Whisper model
- **Input**: Audio data (`audio_common_msgs/AudioData`)
- **Output**: Transcribed text (`std_msgs/String`)

### 2. Natural Language Understanding (NLU)
- **Node**: `nlu_processor.py`
- **Function**: Parses natural language commands and extracts structured meaning
- **Input**: Text commands (`std_msgs/String`)
- **Output**: Structured commands (`capstone_interfaces/CommandStructure`)

### 3. LLM Planner
- **Node**: `llm_planner.py`
- **Function**: Generates detailed task plans from structured commands using LLM reasoning
- **Input**: Structured commands (`capstone_interfaces/CommandStructure`)
- **Output**: Task plans (`capstone_interfaces/TaskPlan`)

### 4. ROS Executor
- **Node**: `ros_executor.py`
- **Function**: Executes task plans using ROS 2 actions and services
- **Input**: Task plans (`capstone_interfaces/TaskPlan`)
- **Output**: Robot actions and execution status (`capstone_interfaces/ExecutionStatus`)

### 5. Integration Node
- **Node**: `integration_node.py`
- **Function**: Coordinates all components and manages system state
- **Input**: All component outputs
- **Output**: System status and coordination

## Message Types

The system uses custom message types defined in the `capstone_interfaces` package:

### CommandStructure.msg
```
Header header
string intent
Entities entities
float32 confidence

# Entities sub-message
message Entities {
  string[] objects
  string[] locations
  string[] actions
}
```

### TaskPlan.msg
```
Header header
string intent
string primary_object
string target_location
PlanStep[] steps
string estimated_time
string success_criteria

# PlanStep sub-message
message PlanStep {
  string id
  string description
  string action_type
  string parameters  # JSON string
}
```

### ExecutionStatus.msg
```
Header header
int32 state  # 0=IDLE, 1=EXECUTING, 2=PAUSED, 3=ERROR, 4=COMPLETED
string message
float32 progress
```

## Launch Configuration

The complete system can be launched using:

```bash
ros2 launch capstone integrated_system.launch.py
```

This launch file includes:
- Voice processing components
- NLU processor
- LLM planner
- ROS executor
- Integration node
- Perception pipeline
- Navigation system
- RViz2 for visualization

## Data Flow

1. **Audio Input**: Microphone captures audio and publishes to `/audio_input`
2. **Transcription**: Whisper converts audio to text, published to `/transcribed_text`
3. **Command Interpretation**: NLU processes text and publishes structured commands to `/structured_command`
4. **Task Planning**: LLM planner generates task plans published to `/task_plan`
5. **Execution**: ROS executor receives plans and executes robot actions
6. **Status Updates**: Execution status published to `/execution_status`

## Safety Features

The system includes multiple safety layers:
- Pre-execution safety checks
- Real-time monitoring during execution
- Emergency stop capabilities
- Validation of planned actions

## Performance Considerations

- Whisper model requires GPU for real-time performance
- LLM planning may have latency depending on API availability
- Execution speed depends on robot capabilities
- Network latency affects system responsiveness

## Troubleshooting

### Common Issues

1. **Audio Not Detected**: Check microphone permissions and audio device configuration
2. **Transcription Errors**: Verify audio quality and noise levels
3. **Plan Generation Failures**: Check LLM API connectivity and rate limits
4. **Execution Failures**: Verify robot state and safety systems

### Debugging

Enable debug output:
```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
ros2 launch capstone integrated_system.launch.py
```

Monitor system status:
```bash
# Listen to system status
ros2 topic echo /system_status

# Monitor individual components
ros2 topic echo /execution_status
ros2 topic echo /llm_debug
```

## Extending the System

The modular design allows for easy extension:
- Add new action types to the executor
- Enhance NLU with more sophisticated parsing
- Improve planning with better LLM prompting
- Add more complex robot behaviors

## Future Improvements

- Real-time optimization of LLM calls
- Improved error recovery mechanisms
- Enhanced safety validation
- Multi-modal planning (vision + language)