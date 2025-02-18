#include <opencv4/opencv2/core.hpp>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_image(py::module_ &m) {
  /// @brief CV Mat, used to hold image and Depth data
  py::class_<cv::Mat>(m, "Mat", py::buffer_protocol())
      .def_buffer([](cv::Mat &m) -> py::buffer_info {
        auto buffer = py::buffer_info();
        buffer.ptr = m.data;                     /* Pointer to buffer */
        buffer.itemsize = sizeof(unsigned char); /* Size of one scalar */
        buffer.format = pybind11::format_descriptor<unsigned char>::
            format();    /* Python struct-style format descriptor */
        buffer.ndim = 3; /* Number of dimensions */
        buffer.shape = {m.rows, m.cols, m.channels()}; /* Buffer dimensions */
        buffer.strides = {
            (long)sizeof(unsigned char) * m.channels() * m.cols,
            (long)sizeof(unsigned char) * m.channels(),
            (long)sizeof(
                unsigned char)}; /* Strides (in bytes) for each index */
        buffer.readonly = true;  /* Buffer is read-write */
        return buffer;
      })
      .def("__repr__",
           [](cv::Mat const &_) -> std::string {
             return fmt::format("Image buffer");
           })
      .def(
          "toImage",
          [](cv::Mat &mat, bool rotate) {
            py::module_ PIL = py::module_::import("PIL");
            py::module_ Image = PIL.attr("Image");
            cv::Mat mat_copy = mat.clone();
            if (rotate)
              cv::rotate(mat_copy, mat_copy, cv::ROTATE_90_CLOCKWISE);
            auto img = Image.attr("frombuffer")(
                "RGB", std::make_tuple(mat_copy.cols, mat_copy.rows),
                py::bytes(reinterpret_cast<const char *>(mat_copy.data),
                          mat_copy.total() * mat_copy.elemSize()));
            return img;
          },
          "rotate"_a = true);
}
