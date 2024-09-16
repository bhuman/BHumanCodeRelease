/**
 * @file SpeedUp.h
 * This file changes some definitions for faster compilation.
 * When compiling the main behavior class, this file must not be included.
 *
 * @author Thomas Röfer
 */

#pragma once

// Reduce module base class to a minimum, keeping the memory layout.
#undef _MODULE_DECLARE_REQUIRES
#define _MODULE_DECLARE_REQUIRES(type) public: const type& the##type;
#undef _MODULE_DECLARE_USES
#define _MODULE_DECLARE_USES(type) public: const type& the##type;
#undef _MODULE_PROVIDES
#define _MODULE_PROVIDES(type, mod) \
  protected: virtual void update(type&) = 0; \
  private: type* _the##type; \
  MessageID _id##type;
#undef _MODULE_I
#define _MODULE_I(name, n, header, ...) _MODULE_II(name, n, header, (_MODULE_PARAMETERS, __VA_ARGS__), (_MODULE_DECLARE, __VA_ARGS__), (__VA_ARGS__))
#undef _MODULE_II
#define _MODULE_II(theName, n, header, params, declare, tail) \
  namespace theName##Module \
  { \
    _MODULE_ATTR_##n params \
    using Parameters = NoParameters; \
  } \
  class theName; \
  class theName##Base : public theName##Module::Parameters \
  _STREAM_UNWRAP header; \
  private: \
    using BaseType = theName##Base; \
  private: \
    _MODULE_ATTR_##n declare \
  public: \
    using Parameters = theName##Module::Parameters; \
    theName##Base(const char*); \
    theName##Base(const theName##Base&) = delete; \
    theName##Base& operator=(const theName##Base&) = delete; \
  }

// The option registration function does not need to be declared inline over and over again.
#undef _CABSL_DECL_CONTEXT__
#define _CABSL_DECL_CONTEXT__(name) \
  _CABSL_DECL_CONTEXT__1(name)
