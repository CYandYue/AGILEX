#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
{
  lidar_type[0] = 1;
  lidar_type[1] = 1;
  point_filter_num[0] = 3;
  point_filter_num[1] = 3;
  N_SCANS[0] = 6;
  N_SCANS[1] = 6;
  SCAN_RATE[0] = 10;
  SCAN_RATE[1] = 10;
  time_unit[0] = 2;
  time_unit[1] = 2;
  blind[0] = 0.1;
  blind[1] = 0.1;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set()
{
  switch (time_unit[0])
  {
    case SEC:
      time_unit_scale[0] = 1.e3f;
      break;
    case MS:
      time_unit_scale[0] = 1.f;
      break;
    case US:
      time_unit_scale[0] = 1.e-3f;
      break;
    case NS:
      time_unit_scale[0] = 1.e-6f;
      break;
    default:
      time_unit_scale[0] = 1.f;
      break;
  }
  switch (time_unit[1])
  {
    case SEC:
      time_unit_scale[1] = 1.e3f;
      break;
    case MS:
      time_unit_scale[1] = 1.f;
      break;
    case US:
      time_unit_scale[1] = 1.e-3f;
      break;
    case NS:
      time_unit_scale[1] = 1.e-6f;
      break;
    default:
      time_unit_scale[1] = 1.f;
      break;
  }
  return;
}

void Preprocess::process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int &lidar_num)
{  
  avia_handler(msg, lidar_num);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int &lidar_num)
{
  switch (lidar_type[lidar_num])
  {
  case OUST64:
    oust64_handler(msg, lidar_num);
    break;

  case VELO16:
    velodyne_handler(msg, lidar_num);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg, const int &lidar_num)
{
  pl_surf.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS[lidar_num]; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  for(uint i=1; i<plsize; i++)
  {
    if((msg->points[i].line < N_SCANS[lidar_num]) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
    {
      valid_num ++;
      if (valid_num % point_filter_num[lidar_num] == 0)
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

        if( ((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
            && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind[lidar_num] * blind[lidar_num])))
        {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int &lidar_num)
{
  pl_surf.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);
  double time_stamp = msg->header.stamp.toSec();
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS[lidar_num] = %d\r\n", plsize, N_SCANS[lidar_num]);
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num[lidar_num] != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
    
    if (range < (blind[lidar_num] * blind[lidar_num])) continue;
    
    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale[lidar_num]; // curvature unit: ms

    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int &lidar_num)
{
    pl_surf.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE[lidar_num];       // scan angular velocity
    std::vector<bool> is_first(N_SCANS[lidar_num],true);
    std::vector<double> yaw_fp(N_SCANS[lidar_num], 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS[lidar_num], 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS[lidar_num], 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    for (int i = 0; i < plsize; i++)
    {
      // if ( (abs(pl_orig.points[i].time) < 1.0 / SCAN_RATE[lidar_num] / 1800) || (abs(pl_orig.points[i].time) > (1.0 / SCAN_RATE[lidar_num]) * 1.1) )
      // {
      //   continue;
      // }
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
      
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale[lidar_num];  // curvature unit: ms // cout<<added_pt.curvature<<endl;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
            yaw_fp[layer]=yaw_angle;
            is_first[layer]=false;
            added_pt.curvature = 0.0;
            yaw_last[layer]=yaw_angle;
            time_last[layer]=added_pt.curvature;
            continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer]=added_pt.curvature;
      }

      if (i % point_filter_num[lidar_num] == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind[lidar_num] * blind[lidar_num]))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
}