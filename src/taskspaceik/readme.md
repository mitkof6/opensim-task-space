# Unimplemented Stuff

* Quaternion derivatives are not smooth due to transitions (see
  https://github.com/simbody/simbody/issues/519) (solved, with two different
  conversations)
* Should handle better bodies that contain two markers
* During walking (gait23), the left foot is not tracked so well, due to noise
  artifacts (should test Kalman Smoothing)
* Properly construct tasks to handle all cases (need more datasets)
