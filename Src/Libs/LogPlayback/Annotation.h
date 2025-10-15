/**
 * @file Annotation.h
 *
 * This file declares a struct that represents an annotation created by
 * the macro ANNOTATION.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/Streamable.h"

struct Annotation : public Streamable
{
  unsigned number = 0; /**< The number of the annotation. */
  unsigned frame = 0; /**< The log frame it belongs to. */
  std::string name; /**< The name of the annotation's source. */
  std::string description; /**< The content of the annotation. */

protected:
  /**
   * Read object from a stream.
   * @param stream The stream to read from. Must be an \c InBinaryMemory that only
   *               contains the data for this object (or at least ends with it).
   */
  void read(In&) override;

  /**
   * Write object to a stream (without \c frame).
   * @param stream The stream to write to.
   */
  void write(Out&) const override;

private:
  /** Registers an incomplete type for this struct, because streaming is non-standard. */
  static void reg();
};
