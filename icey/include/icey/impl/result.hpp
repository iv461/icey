/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.
#pragma once

#include <variant>

namespace icey {
template <class T>
struct Ok {
  explicit Ok(const T &v) : value(v) {}
  T value;
};
template <class T>
struct Err {
  explicit Err(const T &v) : value(v) {}
  T value;
};

/// A result-type is a sum type that holds either a value or error. I is ike Rust's Result and like
/// C++23's std::expected, but for C++20.
template <class _Value, class _Error>
struct Result : std::variant<_Value, _Error> {
  using Value = _Value;
  using Error = _Error;
  Result(const Ok<Value> &ok_value)
      : std::variant<_Value, _Error>(std::in_place_index<0>, ok_value.value) {}
  Result(const Err<Error> &err_value)
      : std::variant<_Value, _Error>(std::in_place_index<1>, err_value.value) {}
  bool has_value() const { return this->index() == 0; }
  bool has_error() const { return this->index() == 1; }
  const Value &value() const { return std::get<0>(*this); }
  const Error &error() const { return std::get<1>(*this); }
  void set_value(const Value &x) { this->template emplace<0>(x); }
  void set_error(const Error &x) { this->template emplace<1>(x); }
  operator bool() const { return has_value(); }
};

}  // namespace icey