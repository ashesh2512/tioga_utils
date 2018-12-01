
#include "MotionBase.h"

namespace tioga_nalu {

std::vector<std::vector<double>> MotionBase::add_motion(
const std::vector<std::vector<double>>& motionL,
const std::vector<std::vector<double>>& motionR)
{
  std::vector<std::vector<double>> comp_trans_mat_(4,std::vector<double>(4,0.0));

  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      for (int k = 0; k < 4; k++) {
        comp_trans_mat_[r][c] += motionL[r][k] * motionR[k][c];
      } // end for loop - k index
    } // end for loop - column index
  } // end for loop - row index

  return comp_trans_mat_;
}

} // tioga_nalu
