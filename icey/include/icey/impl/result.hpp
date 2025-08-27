/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <type_traits>
#include <variant>

namespace icey {
/// A special type that indicates that there is no value. (Using `void` for this would cause many
/// problems, so defining an extra struct is easier.)
struct Nothing {};
/// A tag to be able to recognize the following result type using std::is_base_of_v, a technique we
/// will generally use in the following to recognize (unspecialized) class templates.
struct ResultTag {};
/// A Result-type is a sum type that can either hold Value or Error, or, different
/// to Rust, none. It is used as the state for the Stream.
/// Note that this Result type is  API-compatible with `std::expected` (C++23), so a change is
/// easily possible once we target C++23.
template <class _Value, class _Error>
struct Result : private std::variant<std::monostate, _Value, _Error>, public ResultTag {
  using Value = _Value;
  using Error = _Error;
  using Self = Result<_Value, _Error>;
  static Self None() { return Result<_Value, _Error>{}; }
  static Self Ok(const _Value &x) {
    Self ret;
    ret.template emplace<1>(x);
    return ret;
  }
  static Self Err(const _Error &x) {
    Self ret;
    ret.template emplace<2>(x);
    return ret;
  }
  bool has_none() const { return this->index() == 0; }
  bool has_value() const { return this->index() == 1; }
  bool has_error() const { return this->index() == 2; }
  const Value &value() const { return std::get<1>(*this); }
  const Error &error() const { return std::get<2>(*this); }
  void set_none() { this->template emplace<0>(std::monostate{}); }
  void set_value(const Value &x) { this->template emplace<1>(x); }
  void set_error(const Error &x) { this->template emplace<2>(x); }

  auto get() const {
    if constexpr (std::is_same_v<Error, Nothing>) {
      return this->value();
    } else {
      return *this;
    }
  }
};
}