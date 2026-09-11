# 厂家源码归档

本仓库直接跟踪本机 `src/` 下的厂家源码及本地适配，不再使用 Git
submodule/gitlink。普通 `git clone` 即可获得已归档源码、模型、地图和文档；
编译仍需要 Ubuntu/ROS、系统依赖及本机配置，构建产物与 rosbag 不在版本库中。

`VENDOR_SOURCES.json` 记录转换前每个嵌套仓库的上游 URL、HEAD、本机改动清单
以及历史 gitlink。清单中的改动已包含在本次源码快照中，历史上已删除的文件
保持删除状态；它不是当前工作区的未提交清单。上游许可证保留在各自目录。

归档范围：Livox-SDK2、livox_ros_driver2、navigation、realsense-ros、
rslidar_sdk（包括 rs_driver）、scout_ros、ugv_sdk。FAST-LIO-MULTI 与
ikd-tree 的来源单独记录于 `lio_ws/src/FAST_LIO_MULTI/UPSTREAM.md`。

旧 `.gitmodules` 保存为 `.gitmodules.upstream`，仅作来源参考。转换时
`ugv_sdk/test/googletest` 未下载，因此没有可以归档的本地源码；若需启用该
SDK 的可选测试，应另行准备该依赖，其历史版本记录在 JSON 的 gitlinks 中。

本机原嵌套 Git 元数据已移到
`/tmp/agilex-vendor-git-backup-bjurhlpq`，供短期恢复使用；该临时目录不会推送。
源码快照及版本来源记录保存在主仓库中。
