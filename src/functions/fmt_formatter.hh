#pragma once
#include<fmt/ostream.h>
#include<Eigen/Core>
#include<iostream>


template <typename T>
struct fmt::formatter<
  T,
  std::enable_if_t<
    std::is_base_of_v<Eigen::DenseBase<T>, T>,
    char>> : ostream_formatter {};

// template<>
// struct fmt::formatter<Eigen::Matrix4f>{
//     // Define the parse function (optional for custom types)
//     constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin()) {
//         // No custom parsing, so just return the end of the context
//         return ctx.end();
//     };
    

//     // Define the format function
//     template <typename FormatContext>
//     auto format(const Eigen::Matrix4f& mat, FormatContext& ctx) -> decltype(ctx.out()) {
//         // Convert the Eigen matrix to a string representation
//         std::string result;
//         for (int i = 0; i < mat.rows(); ++i) {
//             for (int j = 0; j < mat.cols(); ++j) {
//                 result += fmt::format("{:12.6f} ", mat(i, j)); // Adjust formatting as needed
//             }
//             result += '\n'; // Add a newline after each row
//         }
//         return fmt::format_to(ctx.out(), "{}", result);
//     }
// };

