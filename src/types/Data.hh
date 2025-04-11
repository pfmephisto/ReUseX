#pragma once
#include <Eigen/Dense>

#include <fmt/format.h>

#include <any>
#include <map>
#include <variant>

#include <opencv4/opencv2/core/mat.hpp>

namespace ReUseX {

enum Field {
  INDEX,
  COLOR,
  DEPTH,
  CONFIDENCE,
  ODOMETRY,
  IMU,
};

template <int F> struct FieldType;

template <> struct FieldType<Field::COLOR> {
  using type = cv::Mat;
};

template <> struct FieldType<Field::DEPTH> {
  using type = Eigen::MatrixXf;
};

template <> struct FieldType<Field::CONFIDENCE> {
  using type = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;
};

template <> struct FieldType<Field::ODOMETRY> {
  using type = Eigen::MatrixXd;
};

template <> struct FieldType<Field::IMU> {
  using type = Eigen::MatrixXd;
};

template <> struct FieldType<Field::INDEX> {
  using type = size_t;
};

//// Helper to extract all types from field_type_map and create a variant
// template <typename... Pairs>
// struct ExtractTypes;
//
// template <typename... Ts>
// struct ExtractTypes<std::pair<Field, Ts>...> {
//     using type = std::variant<Ts...>;
// };
//
//// Define FieldVariant automatically
// using FieldVariant = typename
// ExtractTypes<decltype(field_type_map)::value_type...>::type;

// Define FieldVariant type using std::variant<>
using FieldVariant =
    std::variant<size_t, cv::Mat, Eigen::MatrixXf,
                 Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>,
                 Eigen::MatrixXd, Eigen::Transform<float, 3, Eigen::Affine>>;

class DataItem : public std::map<Field, FieldVariant> {

public:
  template <Field F> typename FieldType<F>::type get() const {
    return std::get<typename FieldType<F>::type>(this->at(F));
  }

  template <Field F> void set(typename FieldType<F>::type value) {
    this->operator[](F) = value;
  }

  std::vector<Field> fields() const {
    std::vector<Field> fields{};
    for (auto const &[key, value] : *this) {
      fields.push_back(key);
    }
    return fields;
  }
};

} // namespace ReUseX

// Specialization of fmt::formatter for ReUseX::Field
template <> struct fmt::formatter<ReUseX::Field> : fmt::formatter<std::string> {
  auto format(ReUseX::Field field, fmt::format_context &ctx) const {
    switch (field) {
    case ReUseX::INDEX:
      return fmt::format_to(ctx.out(), "INDEX");
    case ReUseX::COLOR:
      return fmt::format_to(ctx.out(), "COLOR");
    case ReUseX::DEPTH:
      return fmt::format_to(ctx.out(), "DEPTH");
    case ReUseX::CONFIDENCE:
      return fmt::format_to(ctx.out(), "CONFIDENCE");
    case ReUseX::ODOMETRY:
      return fmt::format_to(ctx.out(), "ODOMETRY");
    case ReUseX::IMU:
      return fmt::format_to(ctx.out(), "IMU");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};
