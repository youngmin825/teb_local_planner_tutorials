define mylaser ranger
(
  sensor
  (
    # just for demonstration purposes
    range [0.1 6] # minimum and maximum range
    fov 360.0 # field of view
    samples 2000 # number of samples
  )
  size [ 0.06 0.15 0.03 ]
)

define myrobot position
(
  
  size [ 0.30 0.30 0.40 ] # (x,y,z)
  localization "gps" # exact localization
  gui_nose 1  # draw nose showing heading
  drive "diff" # diff-drive
  color "red" # red model
  mylaser(pose [ -0.1 0.0 -0.11 0.0 ]) # has mylaser sensor
)
