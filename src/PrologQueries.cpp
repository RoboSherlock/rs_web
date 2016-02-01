#include <SWI-cpp.h>
#include <iostream>

PREDICATE(hello, 1)
{
  std::cout << "Hello baby" << (char *)A1 << std::endl;
  return TRUE;
}
