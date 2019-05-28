/*
 * point_clouds_common.cpp
 *
 *  Created on: Jun 29, 2012
 *      Author: geus
 */

#include<rtcus_compositions/state_composer.h>
#include <rtcus_compositions/impl/point_clouds_common.h>

namespace rtcus_compositions
{

//---------------------- GENERAL ORTOGONAL Point Cloud Compositions --------------------------------------------------------
template<>
  void StateComposer::compose(const sensor_msgs::PointCloud2 & input_cloud, const tf::Transform & new_local_frame,
                              sensor_msgs::PointCloud2& resulting_cloud)
  {

    transform_point_cloud_aux(input_cloud, new_local_frame, resulting_cloud);
  }

template<>
  void StateComposer::inverse_compose(const sensor_msgs::PointCloud2& input_cloud,
                                      const tf::Transform & new_local_frame, sensor_msgs::PointCloud2& resulting_cloud,
                                      const std::string& local_frame_name)
  {

    if (local_frame_name != ANONYMOUS_LOCAL_FRAME_NAME)
      resulting_cloud.header.frame_id = local_frame_name;

    transform_point_cloud_aux(input_cloud, new_local_frame.inverse(), resulting_cloud);
  }
//------------------------------------------------------------------------------------

bool check_field(const sensor_msgs::PointField& input_field, std::string check_name,
                 const sensor_msgs::PointField** output)
{
  bool coherence_error = false;
  if (input_field.name == check_name)
  {
    if (input_field.datatype != sensor_msgs::PointField::FLOAT32
        && input_field.datatype != sensor_msgs::PointField::FLOAT64)
    {
      *output = NULL;
      coherence_error = true;
    }
    else
    {
      *output = &input_field;
    }
  }
  return coherence_error;
}

template<typename T>
  inline void get_float_from_field(const sensor_msgs::PointField* field, unsigned char* data, T& output)
  {
    if (field != NULL)
    {
      if (field->datatype == sensor_msgs::PointField::FLOAT32)
        output = *(reinterpret_cast<const float*>(&data[field->offset]));
      else
        output = (T)(*(reinterpret_cast<const double*>(&data[field->offset])));

    }
    else
      output = 0.0;
  }

template<typename T>
  inline void set_float_to_field(const sensor_msgs::PointField* field, unsigned char* data, const T& value)
  {
    if (field != NULL)
    {
      if (field->datatype == sensor_msgs::PointField::FLOAT32)
        *(reinterpret_cast<float*>(&data[field->offset])) = value;
      else
        *(reinterpret_cast<double*>(&data[field->offset])) = value;
    }
  }

//inspired on http://www.ros.org/doc/api/tf/html/classtf_1_1TransformListener.html#8b9cb24fdcdf596aca647327a666f502
void transform_point_cloud_aux(sensor_msgs::PointCloud2 const& input_cloud, tf::Transform const& transf,
                               sensor_msgs::PointCloud2& resulting_cloud)
{

  bool incompatible_clouds = false;
  const sensor_msgs::PointField *x_field = NULL, *y_field = NULL, *z_field = NULL;

  for (unsigned int i = 0; i < input_cloud.fields.size() && !incompatible_clouds; i++)
  {
    incompatible_clouds |= check_field(input_cloud.fields[i], "x", &x_field);
    incompatible_clouds |= check_field(input_cloud.fields[i], "y", &y_field);
    incompatible_clouds |= check_field(input_cloud.fields[i], "z", &z_field);
  }

  unsigned int length = input_cloud.data.size();

  // Copy relevant data from cloudIn, if needed
  if (&input_cloud != &resulting_cloud)
  {

    resulting_cloud.data.resize(length);
    resulting_cloud.fields.resize(input_cloud.fields.size());
    for (unsigned int i = 0; i < input_cloud.fields.size(); ++i)
      resulting_cloud.fields[i] = input_cloud.fields[i];

    resulting_cloud.height = input_cloud.height;
    resulting_cloud.width = input_cloud.width;
    resulting_cloud.header = input_cloud.header;
    resulting_cloud.point_step = input_cloud.point_step;
    resulting_cloud.row_step = input_cloud.row_step;

    resulting_cloud.header = input_cloud.header;
  }

  // If not, memcpy each group of contiguous fields separately
  for (unsigned int row = 0; row < input_cloud.height; ++row)
  {
    const unsigned char* row_data = &input_cloud.data[row * input_cloud.row_step];
    const unsigned char* out_row_data = &resulting_cloud.data[row * resulting_cloud.row_step];
    for (uint32_t col = 0; col < input_cloud.width; ++col)
    {
      unsigned char* msg_data_input = (unsigned char*)row_data + col * input_cloud.point_step;
      unsigned char* msg_data_output = (unsigned char*)out_row_data + col * resulting_cloud.point_step;

      float x, y, z;
      get_float_from_field(x_field, msg_data_input, x);
      get_float_from_field(y_field, msg_data_input, y);
      get_float_from_field(z_field, msg_data_input, z);

      tf::Vector3 p(x, y, z);
      tf::Vector3 resulting = transf * p;

      set_float_to_field(x_field, msg_data_output, resulting.x());
      set_float_to_field(y_field, msg_data_output, resulting.y());
      set_float_to_field(z_field, msg_data_output, resulting.z());
    }
  }
}
}

