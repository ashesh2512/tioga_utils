
#include "MeshTranslation.h"

#include <cassert>
#include <cmath>

namespace tioga_nalu {

  MeshTranslation::MeshTranslation(
    stk::mesh::MetaData& meta,
    stk::mesh::BulkData& bulk,
    const YAML::Node& node
) : MotionBase(meta, bulk)
{
    load(node);
}

void MeshTranslation::load(const YAML::Node& node)
{
    const auto& fparts = node["mesh_parts"];
    if (fparts.Type() == YAML::NodeType::Scalar) {
        partNames_.push_back(fparts.as<std::string>());
    } else {
        partNames_ = fparts.as<std::vector<std::string>>();
    }

    omega_ = node["omega"].as<double>();
    axis_ = node["axis"].as<std::vector<double>>();
    origin_ = node["origin"].as<std::vector<double>>();

    assert(axis_.size() == 3);
    assert(origin_.size() == 3);
}

void MeshTranslation::initialize(double initial_time)
{
}

void MeshTranslation::execute(double current_time)
{
}

} // tioga_nalu
