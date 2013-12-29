#pragma once

class Power
{
  unsigned char value;
public:
  Power();

  Power(unsigned char value);

  bool isValid();

  operator int();
};
