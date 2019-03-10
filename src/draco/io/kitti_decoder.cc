// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "draco/io/kitti_decoder.h"

#include <fstream>

#include "draco/core/macros.h"
#include "draco/core/status.h"

namespace draco {

KittiDecoder::KittiDecoder()
    : num_positions_(0),
      pos_att_id_(-1),
      int_att_id_(-1),
      out_mesh_(nullptr),
      out_point_cloud_(nullptr) {}

Status KittiDecoder::DecodeFromFile(const std::string &file_name,
                                  Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromFile(file_name, static_cast<PointCloud *>(out_mesh));
}

Status KittiDecoder::DecodeFromFile(const std::string &file_name,
                                  PointCloud *out_point_cloud) {
  std::ifstream file(file_name, std::ios::binary);
  if (!file)
    return Status(Status::IO_ERROR, "Couldn't open file");
  // Read the whole file into a buffer.
  auto pos0 = file.tellg();
  file.seekg(0, std::ios::end);
  auto file_size = file.tellg() - pos0;
  if (file_size == 0)
    return Status(Status::IO_ERROR, "Zero file size");
  file.seekg(0, std::ios::beg);
  std::vector<char> data(file_size);
  file.read(&data[0], file_size);
  buffer_.Init(&data[0], file_size);

  num_positions_ = file_size / (sizeof(float) * 4);
  return DecodeFromBuffer(&buffer_, out_point_cloud);
}

Status KittiDecoder::DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromBuffer(buffer, static_cast<PointCloud *>(out_mesh));
}

Status KittiDecoder::DecodeFromBuffer(DecoderBuffer *buffer,
                                    PointCloud *out_point_cloud) {
  out_point_cloud_ = out_point_cloud;
  buffer_.Init(buffer->data_head(), buffer->remaining_size());
  return DecodeInternal();
}

Status KittiDecoder::DecodeInternal() {
  out_point_cloud_->set_num_points(num_positions_);
  GeometryAttribute pos_va;
  pos_va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
          sizeof(float) * 3, 0);
  pos_att_id_ = out_point_cloud_->AddAttribute(pos_va, true, num_positions_);

  GeometryAttribute int_va;
  int_va.Init(GeometryAttribute::GENERIC, nullptr, 1, DT_FLOAT32, false,
          sizeof(float), 0);
  int_att_id_ = out_point_cloud_->AddAttribute(int_va, true, num_positions_);

  // Parse all lines.
  num_positions_ = 0;
  Status status(Status::OK);
  while (ParseVertexPosition(&status) && status.ok()) {
  }
  if (!status.ok())
    return status;

  // In case there are no faces this is just a point cloud which does
  // not require deduplication.
  if (out_mesh_ && out_mesh_->num_faces() != 0) {
#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
    if (!out_point_cloud_->DeduplicateAttributeValues())
      return Status(Status::ERROR, "Could not deduplicate attribute values");
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
    out_point_cloud_->DeduplicatePointIds();
#endif
  }
  return OkStatus();
}

bool KittiDecoder::ParseVertexPosition(Status *status) {
  // Parse three float numbers for vertex position coordinates
  // and an extra float for the intensity attribute.
  float val[4];
  if (!buffer()->Decode(&val, sizeof(float) * 4))
    return false;
  out_point_cloud_->attribute(pos_att_id_)
      ->SetAttributeValue(AttributeValueIndex(num_positions_), &val[0]);
  out_point_cloud_->attribute(int_att_id_)
      ->SetAttributeValue(AttributeValueIndex(num_positions_), &val[3]);
  ++num_positions_;
  return true;
}

}  // namespace draco
