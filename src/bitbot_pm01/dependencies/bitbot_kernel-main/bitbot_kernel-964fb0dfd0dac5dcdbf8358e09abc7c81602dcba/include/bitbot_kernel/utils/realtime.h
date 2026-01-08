#pragma once

#include "bitbot_kernel/utils/logger.h"

#ifdef __linux__

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>

namespace bitbot
{
  void RtAppStart();
  void RtAppEnd();
}

#endif // __linux__
