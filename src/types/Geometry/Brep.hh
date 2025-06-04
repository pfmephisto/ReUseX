#pragma once
#include "Mesh.hh"
#include "types/Kernel.hh"
#include "types/Point.hh"
#include "types/Point2.hh"

#include <opennurbs_public.h>

namespace ReUseX {
struct Brep : public ON_Brep {
public:
  Brep() = default;
  Brep(Mesh const &mesh);
  void save(std::string const &filename) const;
  static Brep load(std::string const &filename);

  // double volume() const;
  // double area() const;
  // bool is_closed() const;
  // Box get_bbox() const;
  // int get_Orientation() const;
  Mesh get_Mesh() const;
};

} // namespace ReUseX
