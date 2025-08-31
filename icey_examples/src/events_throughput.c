#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <time.h>

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include <rclc/rclc.h>
#include "rclc/executor.h"
#include "rclc/timer.h"

#define TOTAL_TIMERS 100000  // adjust for your machine (1e6 may be too heavy)
#define PERIOD_NS 0    // 1 ms period

static atomic_size_t completed = 0;
static struct timespec start_time, end_time;

// Each timer cancels itself in the callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    rcl_timer_cancel(timer);

    size_t count = atomic_fetch_add(&completed, 1) + 1;
    if (count == TOTAL_TIMERS) {
      clock_gettime(CLOCK_MONOTONIC, &end_time);

      double duration_ms = 
        (end_time.tv_sec - start_time.tv_sec) * 1000.0 +
        (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

      printf("Processed %d timers in %.2f ms (%.0f timers/sec)\n",
             TOTAL_TIMERS,
             duration_ms,
             TOTAL_TIMERS / (duration_ms / 1000.0));
      exit(0);
      rcl_shutdown(NULL);
    }
  }
}

int main(int argc, const char * argv[])
{
  rcl_ret_t rc;
  getchar();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    fprintf(stderr, "Failed to init support: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_node_t node;
  rc = rclc_node_init_default(&node, "c_timer_benchmark", "", &support);
  if (rc != RCL_RET_OK) {
    fprintf(stderr, "Failed to create node: %s\n", rcl_get_error_string().str);
    return 1;
  }

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, TOTAL_TIMERS, &allocator);

  // Allocate timers
  rcl_timer_t * timers = malloc(sizeof(rcl_timer_t) * TOTAL_TIMERS);

  clock_gettime(CLOCK_MONOTONIC, &start_time);

  for (size_t i = 0; i < TOTAL_TIMERS; i++) {
    rc = rclc_timer_init_default(&timers[i], &support, RCL_MS_TO_NS(0), timer_callback);
    if (rc != RCL_RET_OK) {
      fprintf(stderr, "Failed to create timer %zu: %s\n", i, rcl_get_error_string().str);
      return 1;
    }
    rclc_executor_add_timer(&executor, &timers[i]);
  }

  // Spin until shutdown
  rclc_executor_spin(&executor);  

  // Cleanup
  for (size_t i = 0; i < TOTAL_TIMERS; i++) {
    rcl_timer_fini(&timers[i]);
  }
  free(timers);

  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return 0;
}