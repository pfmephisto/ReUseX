// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/ProjectDB.hpp"
#include "core/filter_expression.hpp"
#include "core/logging.hpp"
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace ReUseX::core {

namespace {

// Token types for lexical analysis
enum class TokenType {
  IDENTIFIER, // Cloud name
  NUMBER,     // Integer literal
  IN,         // 'in' keyword
  EQ,         // ==
  NE,         // !=
  GT,         // >
  GE,         // >=
  LT,         // <
  LE,         // <=
  AND,        // &&
  OR,         // ||
  LPAREN,     // (
  RPAREN,     // )
  LBRACKET,   // [
  RBRACKET,   // ]
  COMMA,      // ,
  END         // End of input
};

struct Token {
  TokenType type;
  std::string value;
  size_t position;
};

// Tokenizer
class Tokenizer {
  std::string input_;
  size_t pos_ = 0;

  void skip_whitespace() {
    while (pos_ < input_.size() && std::isspace(input_[pos_])) {
      ++pos_;
    }
  }

  auto peek() const -> char {
    return pos_ < input_.size() ? input_[pos_] : '\0';
  }

  auto advance() -> char {
    return pos_ < input_.size() ? input_[pos_++] : '\0';
  }

public:
  explicit Tokenizer(std::string input) : input_(std::move(input)) {}

  auto next_token() -> Token {
    skip_whitespace();

    if (pos_ >= input_.size()) {
      return {TokenType::END, "", pos_};
    }

    size_t start_pos = pos_;
    char ch = peek();

    // Single-char tokens
    if (ch == '(') {
      advance();
      return {TokenType::LPAREN, "(", start_pos};
    }
    if (ch == ')') {
      advance();
      return {TokenType::RPAREN, ")", start_pos};
    }
    if (ch == '[') {
      advance();
      return {TokenType::LBRACKET, "[", start_pos};
    }
    if (ch == ']') {
      advance();
      return {TokenType::RBRACKET, "]", start_pos};
    }
    if (ch == ',') {
      advance();
      return {TokenType::COMMA, ",", start_pos};
    }

    // Two-char operators
    if (ch == '=' && peek_ahead(1) == '=') {
      advance();
      advance();
      return {TokenType::EQ, "==", start_pos};
    }
    if (ch == '!' && peek_ahead(1) == '=') {
      advance();
      advance();
      return {TokenType::NE, "!=", start_pos};
    }
    if (ch == '>' && peek_ahead(1) == '=') {
      advance();
      advance();
      return {TokenType::GE, ">=", start_pos};
    }
    if (ch == '<' && peek_ahead(1) == '=') {
      advance();
      advance();
      return {TokenType::LE, "<=", start_pos};
    }
    if (ch == '&' && peek_ahead(1) == '&') {
      advance();
      advance();
      return {TokenType::AND, "&&", start_pos};
    }
    if (ch == '|' && peek_ahead(1) == '|') {
      advance();
      advance();
      return {TokenType::OR, "||", start_pos};
    }

    // Single-char comparison operators
    if (ch == '>') {
      advance();
      return {TokenType::GT, ">", start_pos};
    }
    if (ch == '<') {
      advance();
      return {TokenType::LT, "<", start_pos};
    }

    // Numbers
    if (std::isdigit(ch) || ch == '-') {
      std::string num;
      if (ch == '-') {
        num += advance();
      }
      while (std::isdigit(peek())) {
        num += advance();
      }
      return {TokenType::NUMBER, num, start_pos};
    }

    // Identifiers and keywords
    if (std::isalpha(ch) || ch == '_') {
      std::string id;
      while (std::isalnum(peek()) || peek() == '_') {
        id += advance();
      }

      // Check for keywords
      if (id == "in") {
        return {TokenType::IN, id, start_pos};
      }

      return {TokenType::IDENTIFIER, id, start_pos};
    }

    throw std::runtime_error(
        fmt::format("Invalid character '{}' at position {}", ch, start_pos));
  }

private:
  auto peek_ahead(size_t offset) const -> char {
    size_t target = pos_ + offset;
    return target < input_.size() ? input_[target] : '\0';
  }
};

// Parser - Recursive descent with operator precedence
class Parser {
  std::vector<Token> tokens_;
  size_t pos_ = 0;
  ProjectDB &db_;
  std::vector<CloudReferenceNode> clouds_;

  auto current() const -> const Token & {
    return pos_ < tokens_.size() ? tokens_[pos_] : tokens_.back();
  }

  auto peek_type() const -> TokenType { return current().type; }

  auto advance() -> const Token & {
    if (pos_ < tokens_.size() - 1) {
      ++pos_;
    }
    return current();
  }

  auto expect(TokenType type) -> const Token & {
    if (current().type != type) {
      throw std::runtime_error(
          fmt::format("Expected token type {} at position {}, got {}",
                      static_cast<int>(type), current().position,
                      static_cast<int>(current().type)));
    }
    const Token &tok = current();
    advance();
    return tok;
  }

  // Parse expression with OR precedence (lowest)
  auto parse_or() -> std::unique_ptr<FilterNode> {
    auto left = parse_and();

    while (peek_type() == TokenType::OR) {
      advance(); // consume ||
      auto right = parse_and();
      left = std::make_unique<OrNode>(std::move(left), std::move(right));
    }

    return left;
  }

  // Parse expression with AND precedence
  auto parse_and() -> std::unique_ptr<FilterNode> {
    auto left = parse_comparison();

    while (peek_type() == TokenType::AND) {
      advance(); // consume &&
      auto right = parse_comparison();
      left = std::make_unique<AndNode>(std::move(left), std::move(right));
    }

    return left;
  }

  // Parse comparison: cloud op value(s)
  auto parse_comparison() -> std::unique_ptr<FilterNode> {
    // Check for parentheses
    if (peek_type() == TokenType::LPAREN) {
      advance(); // consume (
      auto expr = parse_or();
      expect(TokenType::RPAREN);
      return expr;
    }

    // Expect identifier (cloud name)
    const Token &cloud_tok = expect(TokenType::IDENTIFIER);
    std::string cloud_name = cloud_tok.value;

    // Load cloud if not already loaded
    CloudLConstPtr cloud_ptr = nullptr;
    for (const auto &ref : clouds_) {
      if (ref.name() == cloud_name) {
        cloud_ptr = ref.labels();
        break;
      }
    }

    if (!cloud_ptr) {
      // Load from database
      if (!db_.has_point_cloud(cloud_name)) {
        throw std::runtime_error(
            fmt::format("Label cloud '{}' not found in database", cloud_name));
      }
      cloud_ptr = db_.point_cloud_label(cloud_name);
      clouds_.emplace_back(cloud_name, cloud_ptr);
    }

    // Parse operator
    TokenType op = peek_type();

    if (op == TokenType::IN) {
      advance(); // consume 'in'
      expect(TokenType::LBRACKET);

      std::vector<int32_t> values;
      if (peek_type() != TokenType::RBRACKET) {
        do {
          const Token &num_tok = expect(TokenType::NUMBER);
          values.push_back(std::stoi(num_tok.value));

          if (peek_type() == TokenType::COMMA) {
            advance();
          } else {
            break;
          }
        } while (true);
      }

      expect(TokenType::RBRACKET);
      return std::make_unique<InNode>(std::move(values));
    }

    if (op == TokenType::EQ) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<EqualNode>(std::stoi(val.value));
    }

    if (op == TokenType::NE) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<NotEqualNode>(std::stoi(val.value));
    }

    if (op == TokenType::GT) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<CompareNode>(CompareNode::Op::GT,
                                           std::stoi(val.value));
    }

    if (op == TokenType::GE) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<CompareNode>(CompareNode::Op::GE,
                                           std::stoi(val.value));
    }

    if (op == TokenType::LT) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<CompareNode>(CompareNode::Op::LT,
                                           std::stoi(val.value));
    }

    if (op == TokenType::LE) {
      advance();
      const Token &val = expect(TokenType::NUMBER);
      return std::make_unique<CompareNode>(CompareNode::Op::LE,
                                           std::stoi(val.value));
    }

    throw std::runtime_error(
        fmt::format("Expected comparison operator after '{}' at position {}",
                    cloud_name, current().position));
  }

public:
  Parser(std::vector<Token> tokens, ProjectDB &db)
      : tokens_(std::move(tokens)), db_(db) {
    // Add END token if not present
    if (tokens_.empty() || tokens_.back().type != TokenType::END) {
      tokens_.push_back({TokenType::END, "", 0});
    }
  }

  auto parse() -> std::unique_ptr<FilterExpression> {
    auto expr = std::make_unique<FilterExpression>();
    expr->root = parse_or();
    expr->clouds = std::move(clouds_);

    if (peek_type() != TokenType::END) {
      throw std::runtime_error(
          fmt::format("Unexpected token at position {}", current().position));
    }

    // Validate all clouds have same size
    if (expr->clouds.size() > 1) {
      size_t expected_size = expr->clouds[0].size();
      for (size_t i = 1; i < expr->clouds.size(); ++i) {
        if (expr->clouds[i].size() != expected_size) {
          throw std::runtime_error(fmt::format(
              "Cloud size mismatch: '{}' has {} points, but '{}' has {} points",
              expr->clouds[0].name(), expected_size, expr->clouds[i].name(),
              expr->clouds[i].size()));
        }
      }
    }

    return expr;
  }
};

} // anonymous namespace

auto FilterExpression::evaluate_point(size_t idx) const -> bool {
  // For single-cloud expressions, evaluate directly
  if (clouds.size() == 1) {
    if (idx >= clouds[0].size()) {
      return false;
    }
    int32_t label = clouds[0].labels()->points[idx].label;
    return root->evaluate(label);
  }

  // For multi-cloud expressions, we need to handle per-cloud evaluation
  // This is a simplified implementation - for full support, we'd need to
  // track which cloud each node refers to in the AST
  // For now, we evaluate using the first cloud's label
  // TODO: Implement proper multi-cloud evaluation with cloud context in AST
  if (idx >= clouds[0].size()) {
    return false;
  }
  int32_t label = clouds[0].labels()->points[idx].label;
  return root->evaluate(label);
}

auto parse_filter_expression(const std::string &expression, ProjectDB &db)
    -> std::unique_ptr<FilterExpression> {
  if (expression.empty()) {
    throw std::runtime_error("Empty filter expression");
  }

  // Tokenize
  Tokenizer tokenizer(expression);
  std::vector<Token> tokens;
  Token tok;
  do {
    tok = tokenizer.next_token();
    tokens.push_back(tok);
  } while (tok.type != TokenType::END);

  // Parse
  Parser parser(std::move(tokens), db);
  return parser.parse();
}

auto evaluate_filter(const FilterExpression &expr, size_t cloud_size)
    -> IndicesPtr {
  auto indices = std::make_shared<pcl::Indices>();
  indices->reserve(cloud_size / 2); // Reserve some space

  for (size_t i = 0; i < cloud_size; ++i) {
    if (expr.evaluate_point(i)) {
      indices->push_back(static_cast<pcl::index_t>(i));
    }
  }

  indices->shrink_to_fit();
  return indices;
}

} // namespace ReUseX::core
