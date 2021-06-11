define laser ranger
(
  sensor
  (
    range_min 0.0
    range_max 30.0
    fov 270.25
    samples 1081
  )
  # generic model properties
  color "yellow"
  size [ 0.06 0.15 0.03 ]
)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.7

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  localization "odom"
  odom_error [0.03 0.03 999999 999999 999999 0.2]

  size [ 1.2 0.6 0.6 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 0.0 0.0 0.0 0.0]
  gui_nose 1
  color "blue"

  # kinematics
  drive "car"
  wheelbase 0.68 # distance between both axles
  
  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
