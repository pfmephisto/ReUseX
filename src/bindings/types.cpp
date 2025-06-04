#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_dataset(py::module_ &);
void bind_dataitem(py::module_ &);
void bind_pointcloud(py::module_ &);
void bind_pointclouds(py::module_ &);
void bind_brep(py::module_ &);
void bind_image(py::module_ &);
void bind_filters(py::module_ &);
void bind_pclbase(py::module_ &m);
void bind_visualization(py::module_ &m);

void bind_types(py::module_ &m) {

  bind_dataset(m);  // This is the StrayScanner Dataset
  bind_dataitem(m); // This is one data package from the StrayScanner Dataset

  // PCL Classes
  bind_pclbase(m);
  bind_pointcloud(m); // This is the Point Cloud data type
  bind_pointclouds(m);
  bind_filters(m);

  // Other classes
  bind_brep(m);
  bind_image(m);

  // Visualizer
  bind_visualization(m);
}
