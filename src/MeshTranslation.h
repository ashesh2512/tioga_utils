#ifndef MESHTRANSLATION_H
#define MESHTRANSLATION_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshTranslation : public MotionBase
{
public:
  MeshTranslation(
    stk::mesh::MetaData&,
    const YAML::Node&);

  virtual ~MeshTranslation() {}

  virtual void build_transformation(const double);

private:
  MeshTranslation() = delete;
  MeshTranslation(const MeshTranslation&) = delete;

  void load(const YAML::Node&);

  void translation_mat(const std::vector<double>&);

  std::vector<double> displacement_;
  std::vector<double> velocity_;

  bool use_velocity_;
};


} // tioga_nalu

#endif /* MESHTRANSLATION_H */
