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
        stk::mesh::BulkData&,
        const YAML::Node&);

    virtual ~MeshTranslation() {}

    virtual void initialize(double);

    virtual void execute(double);

private:
    MeshTranslation() = delete;
    MeshTranslation(const MeshTranslation&) = delete;

    void load(const YAML::Node&);

    std::vector<double> origin_{0.0, 0.0, 0.0};

    std::vector<double> axis_;

    double omega_{0.0};
};


} // tioga_nalu

#endif /* MESHTRANSLATION_H */
