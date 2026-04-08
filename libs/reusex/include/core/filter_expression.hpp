// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/types.hpp>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace ReUseX {
// Forward declaration
class ProjectDB;
} // namespace ReUseX

namespace ReUseX::core {

/// Base AST node for filter expressions
class FilterNode {
public:
  virtual ~FilterNode() = default;

  /// Evaluate this node for a specific point's label value
  /// @param label_value The label value to test
  /// @return True if point matches filter condition
  virtual auto evaluate(int32_t label_value) const -> bool = 0;
};

/// Cloud reference node: references a label cloud by name
class CloudReferenceNode {
  std::string cloud_name_;
  CloudLConstPtr label_cloud_;

public:
  CloudReferenceNode(std::string name, CloudLConstPtr labels)
      : cloud_name_(std::move(name)), label_cloud_(std::move(labels)) {}

  auto labels() const -> const CloudLConstPtr & { return label_cloud_; }
  auto name() const -> const std::string & { return cloud_name_; }
  auto size() const -> size_t { return label_cloud_->size(); }
};

/// Equality: <cloud> == value
class EqualNode : public FilterNode {
  int32_t value_;

public:
  explicit EqualNode(int32_t value) : value_(value) {}
  auto evaluate(int32_t label) const -> bool override {
    return label == value_;
  }
};

/// Not equal: <cloud> != value
class NotEqualNode : public FilterNode {
  int32_t value_;

public:
  explicit NotEqualNode(int32_t value) : value_(value) {}
  auto evaluate(int32_t label) const -> bool override {
    return label != value_;
  }
};

/// In-set: <cloud> in [v1, v2, v3]
class InNode : public FilterNode {
  std::unordered_set<int32_t> values_;

public:
  explicit InNode(std::vector<int32_t> values)
      : values_(values.begin(), values.end()) {}
  auto evaluate(int32_t label) const -> bool override {
    return values_.count(label) > 0;
  }
};

/// Comparison operators: <cloud> >= value, etc.
class CompareNode : public FilterNode {
public:
  enum class Op { GT, GE, LT, LE };

private:
  Op op_;
  int32_t value_;

public:
  CompareNode(Op op, int32_t value) : op_(op), value_(value) {}
  auto evaluate(int32_t label) const -> bool override {
    switch (op_) {
    case Op::GT:
      return label > value_;
    case Op::GE:
      return label >= value_;
    case Op::LT:
      return label < value_;
    case Op::LE:
      return label <= value_;
    }
    return false;
  }
};

/// AND combinator: expr1 && expr2
class AndNode : public FilterNode {
  std::unique_ptr<FilterNode> left_;
  std::unique_ptr<FilterNode> right_;

public:
  AndNode(std::unique_ptr<FilterNode> left, std::unique_ptr<FilterNode> right)
      : left_(std::move(left)), right_(std::move(right)) {}
  auto evaluate(int32_t label) const -> bool override {
    return left_->evaluate(label) && right_->evaluate(label);
  }
};

/// OR combinator: expr1 || expr2
class OrNode : public FilterNode {
  std::unique_ptr<FilterNode> left_;
  std::unique_ptr<FilterNode> right_;

public:
  OrNode(std::unique_ptr<FilterNode> left, std::unique_ptr<FilterNode> right)
      : left_(std::move(left)), right_(std::move(right)) {}
  auto evaluate(int32_t label) const -> bool override {
    return left_->evaluate(label) || right_->evaluate(label);
  }
};

/// Parsed filter expression with resolved label cloud references
struct FilterExpression {
  std::unique_ptr<FilterNode> root; ///< AST root
  std::vector<CloudReferenceNode> clouds; ///< Referenced label clouds

  /// Evaluate filter for a point index
  /// @param idx Point index in cloud(s)
  /// @return True if point at idx matches filter
  auto evaluate_point(size_t idx) const -> bool;
};

/// Parse filter expression and resolve label cloud references from ProjectDB
/// @param expression Filter expression string (e.g., "planes in [1, 2]")
/// @param db ProjectDB to load label clouds from
/// @return Parsed expression with resolved cloud references
/// @throws std::runtime_error if syntax invalid or cloud not found
auto parse_filter_expression(const std::string &expression, ProjectDB &db)
    -> std::unique_ptr<FilterExpression>;

/// Evaluate filter expression against all points, generate Indices
/// @param expr Parsed filter expression
/// @param cloud_size Number of points to evaluate
/// @return Indices of points matching filter
auto evaluate_filter(const FilterExpression &expr, size_t cloud_size)
    -> IndicesPtr;

} // namespace ReUseX::core
