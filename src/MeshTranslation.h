#ifndef MESHTRANSLATION_H
#define MESHTRANSLATION_H

#include "MotionBase.h"

#include "yaml-cpp/yaml.h"

namespace tioga_nalu {

class MeshTranslation : public MotionBase
{
public:
  MeshTranslation(
    stk::mesh::MetaData&,
    const YAML::Node&);

    virtual ~MeshTranslation() {}

    virtual void build_transformation(double);

private:
    MeshTranslation() = delete;
    MeshTranslation(const MeshTranslation&) = delete;

    void load(const YAML::Node&);

    std::vector<double> direction_;
};


} // tioga_nalu

#endif /* MESHTRANSLATION_H */
