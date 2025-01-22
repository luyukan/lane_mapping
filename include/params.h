#pragma once
#include <isotream>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace mono_lane_mapping
{
  class LaneMappingParam {
    public:
      typedef std::shared_ptr<LaneMappingParam> Ptr;
      LaneMappingParam();
    private:
  };
} // namespace mono_lane_mapping
