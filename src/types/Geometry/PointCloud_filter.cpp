#include "PointCloud.hh"

#include <vector>

namespace ReUseX {
void PointCloud::filter(std::uint32_t value) {

  // Only keep highest confidence
  ///////////////////////////////////////////////////////////////////////////////
  size_t j = 0;
  for (size_t k = 0; k < (*this)->size(); k++) {
    if ((*this)->at(k).label >= value) {
      (*this)->at(j) = (*this)->at(k);
      j++;
    }
  }
  (*this)->resize(j);
  (*this)->width = j;
  (*this)->height = 1;
  (*this)->is_dense = false;
}
} // namespace ReUseX
