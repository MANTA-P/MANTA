#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <termios.h>

#include "torpedo_control_v2/control_types.hpp"

namespace torpedo_control_v2
{

class KeyboardInput
{
public:
  explicit KeyboardInput(std::size_t queue_capacity);
  ~KeyboardInput();

  KeyboardInput(const KeyboardInput &) = delete;
  KeyboardInput & operator=(const KeyboardInput &) = delete;

  void start();
  void stop();

  // Swaps the event queue under a short lock. The caller owns the returned
  // events and can process them without blocking the input thread.
  std::vector<KeyEvent> drain();

  std::size_t overflow_count() const;

private:
  static double monotonic_now_sec();
  void read_loop();
  void push_event(char key);
  void restore_terminal();

  const std::size_t queue_capacity_;
  mutable std::mutex mutex_;
  std::deque<KeyEvent> events_;
  std::size_t overflow_count_{0U};
  std::atomic<bool> running_{false};
  std::thread thread_;

  int tty_fd_{-1};
  bool terminal_configured_{false};
  termios original_terminal_{};
};

}  // namespace torpedo_control_v2
