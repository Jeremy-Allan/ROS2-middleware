# Automated Hand-Eye Calibration — WIP

## Objective
Develop an automated robot movement sequence for EasyHandEye2
to collect calibration samples using the Kinova Gen3 and Kinect v2.

## Development completed
- Extended EasyHandEye2 robot movement and pose-generation logic.
- Modified the calibration server integration.
- Updated calibration launch configuration.
- Updated package dependencies.
- Investigated Kinova hardware interface command execution.
- Added diagnostic changes to the Kinova hardware interface.

## Testing
- Calibration pose generation was developed and tested.
- MoveIt accepted trajectory requests and reported execution success.
- Physical Kinova arm movement was not successfully verified
  during the final auto-pose testing.
- Controller and hardware interface issues remain unresolved.

## Outstanding work
- Diagnose Kinova command execution and servoing state.
- Validate physical execution of the generated poses.
- Refine pose variation and marker visibility.
- Validate automatic calibration sample collection.
- Test end-to-end calibration accuracy.

## Files
- patches/easy_handeye2.patch
- patches/ros2_kortex.patch
- upstream-versions.txt

## Status
Experimental work in progress. Not validated for physical
robot execution or production use.
