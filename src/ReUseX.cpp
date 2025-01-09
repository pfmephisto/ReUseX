#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/iostream.h>

#include "algorithms/all.hh"
#include "types/all.hh"
#include "functions/all.hh"

#include <eigen3/Eigen/Core>
#include <sstream>
#include <string>
#include <opennurbs_brep.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;
using namespace pybind11::literals;




PYBIND11_MODULE(_core, m) {

    // Header
    m.doc() = R"pbdoc(
        ReUseX - Reuse Explorer
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";


    // Attributes
    //    m.attr("default_params") = ReUseX::plane_fitting_parameters();

    // PYBIND11_NUMPY_DTYPE(ReUseX::Point, x, y, z);
    // PYBIND11_NUMPY_DTYPE(ReUseX::Vector, x, y, z);
    PYBIND11_NUMPY_DTYPE(ReUseX::PointCloud::Cloud::PointType, x, y, z, rgb, normal_x, normal_y, normal_z, curvature, confidence, semantic, instance, label);


    /// Classes

    // Dastatset
    /// @brief A handle for accessing raw data, like the strayscanner export.
    py::class_<ReUseX::Dataset>(m, "Dataset")
        .def(py::init<const std::string &>())
        .def("fields", &ReUseX::Dataset::fields)
        .def("intrinsic_matrix", &ReUseX::Dataset::intrinsic_matrix)
        .def("__bool__", &ReUseX::Dataset::operator bool)
        .def("__getitem__", &ReUseX::Dataset::operator[])
        .def("__getitem__", [](const ReUseX::Dataset &d, py::slice slice) {
            py::list list;
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(d.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            
            for (size_t i = start; i < stop; i += step){
                list.append(d[i]);
            }
            return list;}, "Get a data package", "slice"_a)
        .def("__len__", &ReUseX::Dataset::size)
        .def_property_readonly("size", &ReUseX::Dataset::size)
        .def_property_readonly("color_size", &ReUseX::Dataset::color_size)
        .def_property_readonly("depth_size", &ReUseX::Dataset::depth_size)
        .def_property_readonly("name", &ReUseX::Dataset::name)
        .def("display", &ReUseX::Dataset::display, "Display the dataset",
            "name"_a,
            "show"_a = true )
        ;

    /// @brief Data is the indevidual frames that the dataset provieds.
    /// Think of the data-set as a clollection of data packages.
    py::class_<ReUseX::Data>(m, "Data")
        .def_property_readonly("color", [](const ReUseX::Data & d){return d.get<ReUseX::Field::COLOR>();})
        .def_property_readonly("depth", [](const ReUseX::Data & d){return d.get<ReUseX::Field::DEPTH>();})
        .def_property_readonly("confidence", [](const ReUseX::Data & d){return d.get<ReUseX::Field::CONFIDENCE>();})
        .def_property_readonly("odometry", [](const ReUseX::Data & d){return d.get<ReUseX::Field::ODOMETRY>();})
        .def_property_readonly("imu", [](const ReUseX::Data & d){return d.get<ReUseX::Field::IMU>();})
        .def_property_readonly("pose", [](const ReUseX::Data & d){return d.get<ReUseX::Field::POSES>();})
        ;

    /// @brief The collection of data types a data package can provide.
    /// They are passed to the Dataset on construction to limmit the amount of data that will be loaded.
    py::enum_<ReUseX::Field>(m, "Field")
        .value("COLOR", ReUseX::Field::COLOR)
        // .value("Color", ReUseX::Field::COLOR)
        .value("DEPTH", ReUseX::Field::DEPTH)
        // .value("Depth", ReUseX::Field::DEPTH)
        .value("CONFIDENCE", ReUseX::Field::CONFIDENCE)
        // .value("Confidence", ReUseX::Field::CONFIDENCE)
        .value("ODOMETRY", ReUseX::Field::ODOMETRY)
        // .value("Odometry", ReUseX::Field::ODOMETRY)
        .value("IMU", ReUseX::Field::IMU)
        // .value("Imu", ReUseX::Field::IMU)
        .value("POSES", ReUseX::Field::POSES)
        // .value("Poses", ReUseX::Field::POSES)
        .export_values();

    // py::enum_<ReUseX::Brep::BrepTrim::BrepTrimType>(m, "BrepTrimType")
    //     .value("Unknown", ReUseX::Brep::BrepTrim::BrepTrimType::Unknown)
    //     .value("Boundary", ReUseX::Brep::BrepTrim::BrepTrimType::Boundary)
    //     .value("Mated", ReUseX::Brep::BrepTrim::BrepTrimType::Mated)
    //     .value("Seam", ReUseX::Brep::BrepTrim::BrepTrimType::Seam)
    //     .value("Singular", ReUseX::Brep::BrepTrim::BrepTrimType::Singular)
    //     .value("CurveOnSurface", ReUseX::Brep::BrepTrim::BrepTrimType::CurveOnSurface)
    //     .value("PointOnSurface", ReUseX::Brep::BrepTrim::BrepTrimType::PointOnSurface)
    //     .value("Slit", ReUseX::Brep::BrepTrim::BrepTrimType::Slit)
    //     .export_values();

    // py::enum_<ReUseX::Brep::BrepLoop::BrepLoopType>(m, "BrepLoopType")
    //     .value("Unknown", ReUseX::Brep::BrepLoop::BrepLoopType::Unknown)
    //     .value("Outer", ReUseX::Brep::BrepLoop::BrepLoopType::Outer)
    //     .value("Inner", ReUseX::Brep::BrepLoop::BrepLoopType::Inner)
    //     .value("Slit", ReUseX::Brep::BrepLoop::BrepLoopType::Slit)
    //     .value("CurveOnSurface", ReUseX::Brep::BrepLoop::BrepLoopType::CurveOnSurface)
    //     .value("PointOnSurface", ReUseX::Brep::BrepLoop::BrepLoopType::PointOnSurface)
    //     .export_values();

    /// @brief CV Mat, used to hold image and Depth data
    py::class_<cv::Mat>(m, "Mat", py::buffer_protocol())
        .def_buffer([](cv::Mat &m) -> py::buffer_info {
            auto buffer = py::buffer_info();
            buffer.ptr = m.data;           /* Pointer to buffer */
            buffer.itemsize = sizeof(unsigned char); /* Size of one scalar */
            buffer.format = pybind11::format_descriptor<unsigned char>::format();  /* Python struct-style format descriptor */
            buffer.ndim = 3;          /* Number of dimensions */
            buffer.shape = {  m.rows, m.cols, m.channels() }; /* Buffer dimensions */
            buffer.strides = {
                    (long)sizeof(unsigned char) * m.channels() * m.cols,
                    (long)sizeof(unsigned char) * m.channels(),
                    (long)sizeof(unsigned char)
                }; /* Strides (in bytes) for each index */
            buffer.readonly = true;           /* Buffer is read-write */
            return buffer; 
        })
        ;

    /// @brief Matrix classe used for the other rawd data that the dataset provides.
    typedef Eigen::MatrixXd Matrix;
    typedef Matrix::Scalar Scalar;
    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
    py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
        .def_buffer([](Matrix &m) -> py::buffer_info {
            auto buffer = py::buffer_info();
            buffer.ptr = m.data();           /* Pointer to buffer */
            buffer.itemsize = sizeof(Scalar);/* Size of one scalar */
            buffer.format = py::format_descriptor<Scalar>::format(); /* Python struct-style format descriptor */
            buffer.ndim = 2;          /* Number of dimensions */
            buffer.shape = { m.rows(), m.cols() }; /* Buffer dimensions */
            buffer.strides = {
                    sizeof(Scalar) * (rowMajor ? m.cols() : 1),
                    (long)sizeof(Scalar) * (rowMajor ? 1 : m.rows())
                }; /* Strides (in bytes) for each index */
            buffer.readonly = true;           /* Buffer is read-write */
            return buffer; 
        })
        ;
 
    // TODO: Capture standard out on all functions that use the progress bar
    // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/utilities.html#capturing-standard-output-from-ostream

    /// @brief PointCloud class
    py::class_<ReUseX::PointCloud>(m, "PointCloud", py::buffer_protocol())
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def(py::init([](
            const py::array_t<float> xyz, 
            std::optional<const py::array_t<uint8_t>> rgb, 
            std::optional<const py::array_t<float>> normal, 
            std::optional<const py::array_t<int32_t>> semantic, 
            std::optional<const py::array_t<int32_t>> instance, 
            std::optional<const py::array_t<int32_t>> label) {

                /* Request a buffer descriptor from Python */
                py::buffer_info xyz_info = xyz.request();

                /* Some basic validation checks ... */
                if ( xyz_info.format != py::format_descriptor<float>::format()
                     && xyz_info.format != py::format_descriptor<double>::format())
                    throw std::runtime_error("Incompatible format: expected a float or double array!");

                if (xyz_info.ndim != 2)
                    throw std::runtime_error("Incompatible buffer dimension!");

                auto xyz_ = xyz.unchecked<2>();


                ReUseX::PointCloud cloud;
                (*cloud).points.resize(xyz.shape(0));
                (*cloud).height = xyz.shape(0);
                (*cloud).width = 1;
                
                for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                    (*cloud).points[i].x = *xyz_.data(i, 0);
                    (*cloud).points[i].y = *xyz_.data(i, 1);
                    (*cloud).points[i].z = *xyz_.data(i, 2);
                }

                if (rgb.has_value()){
                    auto rgb_ = rgb.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                        (*cloud).points[i].r = *rgb_.data(i, 2);
                        (*cloud).points[i].g = *rgb_.data(i, 1);
                        (*cloud).points[i].b = *rgb_.data(i, 0);
                    }
                }
                    
                

                if (normal.has_value()){
                    auto normal_ = normal.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                        (*cloud).points[i].normal_x = *normal_.data(i, 0);
                        (*cloud).points[i].normal_y = *normal_.data(i, 1);
                        (*cloud).points[i].normal_z = *normal_.data(i, 2);
                    }
                }
                

                if (semantic.has_value()){
                    auto semantic_ = semantic.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].semantic = *semantic_.data(i, 0);
                }
                    
                

                if (instance.has_value()){
                    auto instance_ = instance.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].instance = *instance_.data(i, 0);
                }
                    
                

                if (label.has_value()){
                    auto label_ = label.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].label = *label_.data(i, 0);
                }
                    
                
                return cloud;
            }),
            "Create Point Cloud from Numpy arrays",
            "xyz"_a, 
            "rgb"_a = py::none(), 
            "normal"_a = py::none(), 
            "semantic"_a = py::none(), 
            "instance"_a = py::none(), 
            "label"_a = py::none(),
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &ReUseX::PointCloud::load,
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("save", &ReUseX::PointCloud::save, 
            "Save a point cloud to disk",
            "output_file"_a,
            "binary"_a = true, py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &ReUseX::PointCloud::filter, 
            "Filter the point cloud", 
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("downsample", &ReUseX::PointCloud::downsample, 
            "Downsample the point cloud", 
            "leaf_size"_a=0.02, 
            py::return_value_policy::reference,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("region_growing", &ReUseX::PointCloud::region_growing, 
            "Region growing",
            "angle_threshold"_a = float(0.96592583), // cos(25°)
            "plane_dist_threshold"_a = float(0.1),
            "minClusterSize"_a = int(2*(1/0.02)*(1/0.02)),
            "early_stop"_a = float(0.3),
            "radius"_a = float(0.1),
            "interval_0"_a = float(16),
            "interval_factor"_a = float(1.5),
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("clustering", &ReUseX::PointCloud::clustering, "Cluster the sematic labels in to instances",
            "cluster_tolerance"_a = 0.02,
            "min_cluster_size"_a = 100,
            "max_cluster_size"_a = std::numeric_limits<pcl::uindex_t>::max(),
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("solidify", &ReUseX::PointCloud::solidify, 
            "Solidify the point cloud",
            "downsample_size"_a = 5000000,
            "sx"_a = 0.4,
            "sy"_a = 0.4,
            "expand_factor"_a = 2,
            "inflate_factor"_a = 1.2,
            "max_loop"_a = 10.0,
            "mult_factor"_a = 1.0,
            "fitting"_a = 0.20,
            "coverage"_a = 0.10,
            "complexity"_a = 0.70,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display",&ReUseX::PointCloud::display, 
            "Display the point cloud",
            "name"_a = "Cloud", 
            py::return_value_policy::reference_internal
        )
        .def("bbox", &ReUseX::PointCloud::get_bbox, 
            "Get the bounding box of the point cloud"
        )
        .def("__len__", [](ReUseX::PointCloud &cloud){ return cloud->size();})
        .def_buffer([](ReUseX::PointCloud &cloud) -> py::buffer_info {
            return py::buffer_info(
                cloud->points.data(),
                sizeof(ReUseX::PointCloud::Cloud::PointType),
                py::format_descriptor<ReUseX::PointCloud::Cloud::PointType>::format(),
                1,
                { cloud->points.size() },
                { sizeof(ReUseX::PointCloud::Cloud::PointType)}
            );
        })
        ;

    /// @brief Collection of point clouds in memory class
    py::class_<ReUseX::PointCloudsInMemory>(m, "PointCloudsInMemory")
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &ReUseX::PointCloudsInMemory::load, 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &ReUseX::PointCloudsInMemory::filter, 
            "Filter the point cloud",
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("register", &ReUseX::PointCloudsInMemory::register_clouds, 
            "Register the point clouds", 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("merge", &ReUseX::PointCloudsInMemory::merge, 
            "Merge the point clouds",
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("annotate", &ReUseX::PointCloudsInMemory::annotate, 
            "Annotate the point clouds",
            "yolo_path"_a,
            "dataset"_a = py::none(),
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display", &ReUseX::PointCloudsInMemory::display, 
            "Display the point clouds",
            "show_clouds"_a = false, 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("__getitem__", [](const ReUseX::PointCloudsInMemory &obj, std::size_t index) {
            return obj[index]; }, 
            "Get a point cloud", 
            "index"_a
        )
        .def("__getitem__", [](const ReUseX::PointCloudsInMemory &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, 
            "Get a point cloud", 
            "slice"_a
        )
        .def("__len__", &ReUseX::PointCloudsInMemory::size)
        ; 

    /// @brief Collection of point clouds on disk class
    py::class_<ReUseX::PointCloudsOnDisk>(m, "PointCloudsOnDisk")
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &ReUseX::PointCloudsOnDisk::load, 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &ReUseX::PointCloudsOnDisk::filter, 
            "Filter the point cloud",
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("register", &ReUseX::PointCloudsOnDisk::register_clouds, 
            "Register the point clouds", 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("merge", &ReUseX::PointCloudsOnDisk::merge, 
            "Merge the point clouds",
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("annotate", &ReUseX::PointCloudsOnDisk::annotate, 
            "Annotate the point clouds",
            "yolo_path"_a, 
            "dataset"_a = py::none(), 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display", &ReUseX::PointCloudsOnDisk::display, 
            "Display the point clouds",
            "show_clouds"_a = false,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("__getitem__", [](const ReUseX::PointCloudsOnDisk &obj, std::size_t index) {
            return obj[index]; }, 
            "Get a point cloud", 
            "index"_a
        )
        .def("__getitem__", [](const ReUseX::PointCloudsOnDisk &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, 
            "Get a point cloud", 
            "slice"_a
        )
        .def("__len__", &ReUseX::PointCloudsOnDisk::size)
        ;

    /// @brief Plane class
    py::class_<ReUseX::Plane>(m, "Plane")
        .def(py::init<>())
        .def(py::init<const float ,const float ,const float ,const float>())
        .def(py::init<const float ,const float ,const float ,const float, const float, const float, const float>())
        .def("__repr__", [](const ReUseX::Plane &p){
            std::stringstream ss;
            ss << "Plane A("<< p.a()<<") B(" <<p.b() << ") C(" <<p.c() << ") D(" << p.d() << ")";
            return ss.str();
        })
        .def_readwrite("origin", &ReUseX::Plane::origin)
        .def("normal", &ReUseX::Plane::normal)
        ;
 
    /// @brief Position
    py::class_<ReUseX::Point>(m, "Pos")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const ReUseX::Point &p){
            std::stringstream ss;;
            ss << "Pos X=" << p.x() << " y="<< p.y() << " z=" << p.z();
            return ss.str();
        })
        .def("x", &ReUseX::Point::x)
        .def("y", &ReUseX::Point::y)
        .def("z", &ReUseX::Point::z)
        ;

    /// @brief Position
    py::class_<ReUseX::Point2>(m, "Pos2D")
        .def(py::init<const float, const float>())
        .def(("__repr__"), [](const ReUseX::Point2 &p){
            std::stringstream ss;;
            ss << "Pos2D X=" << p.x() << " y="<< p.y();
            return ss.str();
        })
        .def("x", &ReUseX::Point2::x)
        .def("y", &ReUseX::Point2::y)
        ;


    /// @brief Vector
    py::class_<ReUseX::Vector>(m, "Vec")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const ReUseX::Vector &v){
            std::stringstream ss;;
            ss << "Vec X=" << v.x() << " y="<< v.y() << " z=" << v.z();
            return ss.str();
        })
        ;

    /// @brief Direction, Normalised vector
    py::class_<ReUseX::Direction>(m, "Dir")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const ReUseX::Direction &v){
            std::stringstream ss;;
            ss << "Dir X=" << v.dx() << " y="<< v.dy() << " z=" << v.dz();
            return ss.str();
        })
        // .def_property_readonly("valid", [](const ReUseX::Direction &v){ return tg::normalize_safe((ReUseX::Vector)v) !=  ReUseX::Vector::zero;
        //  })
        ;
    py::class_<ReUseX::AABB>(m, "AABB")
        .def(py::init<const ReUseX::Point, const ReUseX::Point>())
        .def("__repr__", [](const ReUseX::AABB &a){
            std::stringstream ss;
            ss << "AABB min(" << a.min().x() << ", " << a.min().y() << ", " << a.min().z() << ") max(" << a.max().x() << ", " << a.max().y() << ", " << a.max().z() << ")";
            return ss.str();
        })
        .def("volume", [](const ReUseX::AABB &a){
            return (a.max().x() - a.min().x()) * (a.max().y() - a.min().y()) * (a.max().z() - a.min().z());
        })
        .def("center", [](const ReUseX::AABB &a){
            return ReUseX::Point((a.max().x() + a.min().x()) / 2, (a.max().y() + a.min().y()) / 2, (a.max().z() + a.min().z()) / 2);
        })
        .def("xInterval", [](const ReUseX::AABB &a){
            return std::make_tuple(a.min().x(), a.max().x());
        })
        .def("yInterval", [](const ReUseX::AABB &a){
            return std::make_tuple(a.min().y(), a.max().y());
        })
        .def("zInterval", [](const ReUseX::AABB &a){
            return std::make_tuple(a.min().z(), a.max().z());
        })
        ;

    // Create compatible class for the ON_BoundingBox class
    // py::class_<ON_BoundingBox>(m, "BoundingBox")
    //     .def(py::init<const ReUseX::Point, const ReUseX::Point>())
    //     .def("__repr__", [](const ReUseX::AABB &a){
    //         std::stringstream ss;
    //         ss << "AABB min(" << a.min().x() << ", " << a.min().y() << ", " << a.min().z() << ") max(" << a.max().x() << ", " << a.max().y() << ", " << a.max().z() << ")";
    //         return ss.str();
    //     })
    //     .def("volume", [](const ReUseX::AABB &a){
    //         return (a.max().x() - a.min().x()) * (a.max().y() - a.min().y()) * (a.max().z() - a.min().z());
    //     })
    //     .def("center", [](const ReUseX::AABB &a){
    //         return ReUseX::Point((a.max().x() + a.min().x()) / 2, (a.max().y() + a.min().y()) / 2, (a.max().z() + a.min().z()) / 2);
    //     })
    //     .def("xInterval", [](const ReUseX::AABB &a){
    //         return std::make_tuple(a.min().x(), a.max().x());
    //     })
    //     .def("yInterval", [](const ReUseX::AABB &a){
    //         return std::make_tuple(a.min().y(), a.max().y());
    //     })
    //     .def("zInterval", [](const ReUseX::AABB &a){
    //         return std::make_tuple(a.min().z(), a.max().z());
    //     })
    //     ;
    
    py::class_<ReUseX::Brep>(m, "Brep")
        .def("save", &ReUseX::Brep::save)
        .def_static("load", &ReUseX::Brep::load, py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>())
        // .def("volume", &ReUseX::Brep::volume)
        // .def("area", &ReUseX::Brep::area)
        // .def("bbox", &ReUseX::Brep::m_bbox)
        .def("is_closed", &ReUseX::Brep::IsSolid)

        // .def("Curves2D", [](const ReUseX::Brep &b){
        //     std::vector<std::vector<std::array<double, 2>>> curves = std::vector<std::vector<std::array<double, 2>>>(b.m_C2.Count());
        //     for (int i = 0; i < b.m_C2.Count(); i++){
        //         ON_Curve curve = b.m_C2.operator[0] ;

        //         for (int j = 0; j < curve.Count(); j++){
        //             auto point = b.m_C2[i][j];
        //             auto c = b.m_C2[i][j];
        //             curves[i].push_back({c[0], c[1]});
        //         }
                    
        //         auto c = b.m_C2[i];
        //         curves[i] = {c[0], c[1]};
        //     }
        //     return curves;
        // })
        // .def("Curves3D", [](const ReUseX::Brep &b){
        //     std::vector<std::array<double, 3>>curves = std::vector<std::array<double, 3>>(b.m_C3.Count());
        //     for (int i = 0; i < b.m_C3.Count(); i++){
        //         auto c = b.m_C3[i];
        //         curves[i] = {c[0], c[1], c[2]};
        //     }
        //     return curves;
        // })

        // .def("Edges", [](const ReUseX::Brep &b){
        //     std::vector<ON_BrepEdge> edges = std::vector<ON_BrepEdge>(b.m_E.Count());
        //     for (int i = 0; i < b.m_E.Count(); i++){
        //         auto e = b.m_E[i];
        //         edges[i] = e;
        //     }
        // })
        // .def("Faces", [](const ReUseX::Brep &b){
        //     std::vector<ON_BrepFace> faces = std::vector<ON_BrepFace>(b.m_F.Count());
        //     for (int i = 0; i < b.m_F.Count(); i++){
        //         auto f = b.m_F[i];
        //         faces[i] = f;
        //     }
        //     return face;
        // })
        // .def("Vertices", [](const ReUseX::Brep &b){
        //     std::vector<std::array<double, 3>> vertices = std::vector<std::array<double, 3>>(b.m_V.Count());

        //     for (int i = 0; i < b.m_V.Count(); i++){
        //         auto v = b.m_V[i];
        //         vertices[i] = {v[0], v[1], v[2]};
        //     }
            
        //     return vertices;
        // })

        // .def("Surfaces", [](const ReUseX::Brep &b){
        //     std::vector<ON_Surface> surfaces = std::vector<ON_Surface>(b.m_S.Count());
        //     for (int i = 0; i < b.m_S.Count(); i++){
        //         auto s = b.m_S[i];
        //         surfaces[i] = s;
        //     }
        //     return surfaces;
        // })
        // .def("Loops", [](const ReUseX::Brep &b){
        //     std::vector<ON_BrepLoop> loops = std::vector<ON_BrepLoop>(b.m_L.Count());
        //     for (int i = 0; i < b.m_L.Count(); i++){
        //         auto l = b.m_L[i];
        //         loops[i] = l;
        //     }
        //     return loops;
        // })
        // .def("Trims", [](const ReUseX::Brep &b){
        //     std::vector<ON_BrepTrim> trims = std::vector<ON_BrepTrim>(b.m_T.Count());
        //     for (int i = 0; i < b.m_T.Count(); i++){
        //         auto t = b.m_T[i];
        //         trims[i] = t;
        //     }
        //     return trims;
        // })
        .def("Orientation", &ReUseX::Brep::SolidOrientation) //solid_orientation
        .def("get_mesh", &ReUseX::Brep::get_Mesh)
        .def("display", &ReUseX::Brep::display, 
            "Display the Brep", 
            "name"_a = "Brep", 
            "show_mesh"_a = true)
        ;

    py::class_<ON_BrepFace>(m, "Face")
        .def(py::init<>())
        .def_readwrite("SurfaceIndex", &ON_BrepFace::m_si)
        .def("OuterLoopIndex", [](const ON_BrepFace &bf ){return bf.m_li[0];})
        .def_readwrite("OrientationReversed", &ON_BrepFace::m_bRev)
        .def_readwrite("LoopIndices", &ON_BrepFace::m_li)
        ;
    
    py::class_<ON_BrepEdge>(m, "Edge")
        .def(py::init<>())
        .def_readonly("Curve3dIndex", &ON_BrepEdge::m_c3i)
        .def_readonly("TrimIndices", &ON_BrepEdge::m_ti)
        .def("StartIndex", [](const ON_BrepEdge &be){ return be.m_vi[0];})
        .def("EndIndex",   [](const ON_BrepEdge &be){ return be.m_vi[1];})
        .def("ProxyCurveIsReversed", &ON_BrepEdge::ProxyCurveIsReversed)
        .def("Domain", &ON_BrepEdge::Domain)
        ;


        py::class_<ON_NurbsSurface>(m, "NurbsSurface")
        .def(py::init<>())
        .def("degreeU", [](const ON_NurbsSurface &s){return s.Degree(0);})
        .def("degreeV", [](const ON_NurbsSurface &s){return s.Degree(1);})
        .def("rational", &ON_NurbsSurface::IsRational)
        // .def_readwrite("area", &ON_NurbsSurface::area)
        .def_readwrite("pointData", &ON_NurbsSurface::m_cv)
        .def("countU", []( const ON_NurbsSurface &s){return s.m_cv_count[0];})
        .def("countV", []( const ON_NurbsSurface &s){return s.m_cv_count[1];})
        // .def_readwrite("bbox", &ON_NurbsSurface::BoundingBox)
        .def("closedU", [](const ON_NurbsSurface &s){s.IsClosed(0);})
        .def("closedV", [](const ON_NurbsSurface &s){s.IsClosed(1);})
        .def("domainU", [](const ON_NurbsSurface &s){return s.Domain(0);})
        .def("domainV", [](const ON_NurbsSurface &s){return s.Domain(1);})
        .def("knotsU", [](const ON_NurbsSurface &s){
            auto count = s.KnotCount(0);
            std::vector<double> knots(count);
            for (int i = 0; i < count; i++)
                knots[i] = s.Knot(0, i);
            return knots;
            }
        )
        .def("knotsV", [](const ON_NurbsSurface &s){
            auto count = s.KnotCount(1);
            std::vector<double> knots(count);
            for (int i = 0; i < count; i++)
                knots[i] = s.Knot(1, i);
            return knots;
            }
        )
        ;

        py::class_<ON_Extrusion>(m, "Extrusion")
        .def(py::init<>())
        .def("GetNurbsForm", [](const ON_Extrusion &s){
            ON_NurbsSurface ns;
            s.GetNurbForm(ns, 0.00001);
            return ns;
            }
        )
        ;

        py::class_<ON_PlaneSurface>(m, "PlaneSurface")
        .def(py::init<>())
        .def("GetNurbsForm", [](const ON_PlaneSurface &s){
            ON_NurbsSurface ns;
            s.GetNurbForm(ns, 0.00001);
            return ns;
            }
        )
        ;

    py::class_<ON_BrepTrim>(m, "BrepTrim")
        .def(py::init<>())
        .def_readwrite("EdgeIndex", &ON_BrepTrim::m_ei)
        .def("StartIndex",[](const ON_BrepTrim& bt){ return bt.m_vi[0];})
        .def("EndIndex",  [](const ON_BrepTrim& bt){ return bt.m_vi[1];})
        .def_readwrite("FaceIndex", &ON_BrepTrim::m_trim_index)
        .def_readwrite("LoopIndex", &ON_BrepTrim::m_li)
        .def_readwrite("CurveIndex", &ON_BrepTrim::m_c2i)
        .def_readwrite("IsoStatus", &ON_BrepTrim::m_iso)
        .def_readwrite("TrimType", &ON_BrepTrim::m_type)
        .def_readwrite("IsReversed", &ON_BrepTrim::m_bRev3d)
        // .def_readwrite("Domain", &ON_BrepTrim::Domain)
        ;   
    py::class_<ON_BrepLoop>(m, "BrepLoop")
        .def(py::init<>())
        .def_readwrite("FaceIndex", &ON_BrepLoop::m_fi)
        .def_readwrite("TrimIndices", &ON_BrepLoop::m_ti)
        .def_readwrite("Type", &ON_BrepLoop::m_type)
        ;

    py::class_<ON_Interval>(m, "Domain")
        .def(py::init<>())
        .def("Min", [](const ON_Interval &i){i.m_t[0];})
        .def("Max", [](const ON_Interval &i){i.m_t[1];})
        ;


    py::class_<ReUseX::Mesh>(m, "Mesh")
        .def(py::init<>())
        .def("volume", &ReUseX::Mesh::volume)
        .def("area", &ReUseX::Mesh::area)
        .def("bbox", &ReUseX::Mesh::get_bbox)
        .def("vertices", &ReUseX::Mesh::get_vertices)
        .def("faces", &ReUseX::Mesh::get_faces)
        .def("colors", &ReUseX::Mesh::get_colors)
        .def("textrueCoords", &ReUseX::Mesh::get_textrueCoords)
        ;




    // TODO: Those should be moved to their respecive classes
    // Functions
    m.def("parse_dataset", &ReUseX::parse_Dataset, "Parse a StrayScanner scan in to a point cloud"
        "dataset"_a,
        "output_path"_a,
        "start"_a = 0,
        "stop"_a = nullptr,
        "step"_a = 5
    );

    m.def("extract_instances", &ReUseX::extract_instances, "Extract the instance for a point cloud"
        "point_cloud"_a);




#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif    
    
} // PYBIND11_MODULE(ReUseX, m) 