/**
 * @file Function.h
 *
 * This file defines a macro that encapsulates functions. The main difference from
 * std::function is that no exception is generated if the wrapper is called with no function
 * assigned. Instead, nothing happens. If the function is supposed to return data, a
 * default value is returned instead, i.e. the value created by the default constructor of
 * the return type. For Eigen types, the result of Zero() is returned instead.
 * The macro also ensures that the blackboard is able to identify representations that
 * contains functions. Thereby, it can reset these representations when their provider
 * changes.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <functional>
#include <utility>

namespace FunctionImpl
{
  template<typename S> class Function;

  template<typename R, typename... A> class Function<R(A...)> : public std::function<R(A...)>
  {
  private:
    /**
     * Returns the result of Zero() if a class has such a method, otherwise a default
     * object of the class is returned.
     * @param T The type of the class.
     */
    template<typename T> static auto zero(T*, T*) -> decltype(T::Zero(), T()) {return T::Zero();}
    template<typename T> static T zero(T*, void*) {return T();}

  public:
    using std::function<R(A...)>::function;

    R operator()(A... args) const
    {
      if(*this)
        return std::function<R(A...)>::operator()(std::forward<A>(args)...);
      else
        return zero(static_cast<R*>(nullptr), static_cast<R*>(nullptr));
    }
  };
}

/**
 * Declare the type of a function. For instance: FUNCTION(void()) fn;
 * @param ... The type of the function.
 */
#define FUNCTION(...) friend struct HasSerialize; FunctionImpl::Function<__VA_ARGS__>

/**
 * If a class is derived from a class that has functions, but the derived
 * class does not declare new functions, but overrides "serialize", this macro must be used
 * inside the class' body. If a class is derived using STREAMABLE_WITH_BASE, "serialize"
 * is automatically overridden, i.e. this macro must be used if a base class contains functions.
 */
#define BASE_HAS_FUNCTION friend struct HasSerialize;
