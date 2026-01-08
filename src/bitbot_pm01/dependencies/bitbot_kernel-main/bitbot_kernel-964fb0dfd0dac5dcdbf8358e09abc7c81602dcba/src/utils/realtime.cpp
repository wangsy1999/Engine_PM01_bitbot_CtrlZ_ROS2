#include "bitbot_kernel/utils/realtime.h"
#include "bitbot_kernel/utils/logger.h"

#ifdef __linux__

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>

namespace
{
  static int latency_target_fd = -1;

  void SetCPULatency(void)
  {
    auto logger = bitbot::Logger().ConsoleLogger();

    int32_t latency_target_value = 0;
    struct stat s;
    int ret;
    if (stat("/dev/cpu_dma_latency", &s) == 0) {
      latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
      if (latency_target_fd == -1)
        return;
      ret = write(latency_target_fd, &latency_target_value, 4);
      if (ret == 0) {
        logger->error("error setting cpu_dma_latency to {}!: {}", latency_target_value, strerror(errno));
        close(latency_target_fd);
        return;
      }
      logger->debug("/dev/cpu_dma_latency set to {}us", latency_target_value);
    }
  }

  void LockMemory()
  {
    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
      bitbot::Logger().ConsoleLogger()->error("mlockall failed");
    }    
  }
}

namespace bitbot
{
  void RtAppStart()
  {
    LockMemory();

    SetCPULatency();
  }

  void RtAppEnd()
  {
    close(latency_target_fd);
  }
}

#endif // __linux__
