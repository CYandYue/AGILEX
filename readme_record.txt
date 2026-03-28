roslaunch scout_bringup open_MID360lidar.launch
roslaunch scout_bringup gmapping.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true
# 一键模式（录制+自动提取）                                                                               
  python3 ~/agilex_ws/collect_data_v2.py ~/my_dataset       
                                                                                                            
  # 仅提取已有 bag（无需重新录制）                                                                          
  python3 ~/agilex_ws/collect_data_v2.py extract ~/my_dataset.bag ~/my_dataset
