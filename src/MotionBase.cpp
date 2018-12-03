
#include "MotionBase.h"

namespace tioga_nalu {

MotionBase::trans_mat_type MotionBase::add_motion(
    const trans_mat_type& motionL,
    const trans_mat_type& motionR)
{
  trans_mat_type comp_trans_mat_ = {};

  for (int r = 0; r < trans_mat_size; r++) {
    for (int c = 0; c < trans_mat_size; c++) {
      for (int k = 0; k < trans_mat_size; k++) {
        comp_trans_mat_[r][c] += motionL[r][k] * motionR[k][c];
      } // end for loop - k index
    } // end for loop - column index
  } // end for loop - row index

  return comp_trans_mat_;
}

} // tioga_nalu
