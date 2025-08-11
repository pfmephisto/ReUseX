#pragma once
// #include <Eigen/Core/util/ForwardDeclarations.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/printf.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#include <opencv4/opencv2/core.hpp>

// template<>
// struct fmt::formatter<Eigen::Matrix4f>{
//     // Define the parse function (optional for custom types)
//     constexpr auto parse(fmt::format_parse_context& ctx) ->
//     decltype(ctx.begin()) {
//         // No custom parsing, so just return the end of the context
//         return ctx.end();
//     };

//     // Define the format function
//     template <typename FormatContext>
//     auto format(const Eigen::Matrix4f& mat, FormatContext& ctx) ->
//     decltype(ctx.out()) {
//         // Convert the Eigen matrix to a string representation
//         std::string result;
//         for (int i = 0; i < mat.rows(); ++i) {
//             for (int j = 0; j < mat.cols(); ++j) {
//                 result += fmt::format("{:12.6f} ", mat(i, j)); // Adjust
//                 formatting as needed
//             }
//             result += '\n'; // Add a newline after each row
//         }
//         return fmt::format_to(ctx.out(), "{}", result);
//     }
// };
//
//
const Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "",
                                "\n[", "]");

// Parese Eigen matrix with included formatter
template <typename T>
struct fmt::formatter<Eigen::WithFormat<T>> : fmt::ostream_formatter {};

template <typename T>
struct fmt::formatter<cv::Size_<T>> : fmt::ostream_formatter {};

template <typename Scalar>
struct fmt::formatter<Eigen::Quaternion<Scalar>>
    : fmt::formatter<fmt::string_view> {
  auto format(const Eigen::Quaternion<Scalar> &value, format_context &ctx) const
      -> format_context::iterator {
    return fmt::format_to(ctx.out(), "Ouat[{:.3f}X {:.3f}Y {:.3f}Z {:.3f}W]",
                          value.x(), value.y(), value.z(), value.w());
  }
};

template <>
struct fmt::formatter<Eigen::Matrix<float, 4, 1>>
    : fmt::formatter<fmt::string_view> {

  auto format(const Eigen::Matrix<float, 4, 1> &value,
              format_context &ctx) const -> format_context::iterator {
    return fmt::format_to(ctx.out(), "Vec[{:.3f},{:.3f},{:.3f},{:.3f}]",
                          value(0, 0), value(1, 0), value(2, 0), value(3, 0));
  }
};

// template <typename Scalar> struct fmt::formatter<Eigen::MatrixX<Scalar>> {
//
//   // The parse function can be used to parse any format options (unused here)
//   constexpr auto parse(fmt::format_parse_context &ctx) {
//     // Parsing is not needed for simple vector printing, so we return the end
//     // iterator
//     return ctx.end();
//   }
//
//   // The format function is responsible for formatting the vector
//   template <typename FormatContext>
//   auto format(const Eigen::MatrixX<Scalar> &quat, FormatContext &ctx) {
//     return "Matrix";
//   }
// };

template <typename T>
struct fmt::formatter<
    T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter {};
