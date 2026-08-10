#include "torpedo_control_v2/keyboard_input.hpp"

#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace torpedo_control_v2
{

KeyboardInput::KeyboardInput(const std::size_t queue_capacity)
: queue_capacity_(queue_capacity)
{
  tty_fd_ = ::open("/dev/tty", O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (tty_fd_ < 0) {
    throw std::runtime_error(
            std::string("Cannot open /dev/tty for keyboard input: ") + std::strerror(errno));
  }

  if (::tcgetattr(tty_fd_, &original_terminal_) != 0) {
    const std::string error =
      std::string("Cannot read terminal settings: ") + std::strerror(errno);
    ::close(tty_fd_);
    tty_fd_ = -1;
    throw std::runtime_error(error);
  }

  termios raw_terminal = original_terminal_;
  raw_terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw_terminal.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
  raw_terminal.c_cc[VMIN] = 0;
  raw_terminal.c_cc[VTIME] = 0;

  if (::tcsetattr(tty_fd_, TCSANOW, &raw_terminal) != 0) {
    const std::string error =
      std::string("Cannot configure terminal: ") + std::strerror(errno);
    ::close(tty_fd_);
    tty_fd_ = -1;
    throw std::runtime_error(error);
  }
  terminal_configured_ = true;
}

KeyboardInput::~KeyboardInput()
{
  stop();
  restore_terminal();
  if (tty_fd_ >= 0) {
    (void)::close(tty_fd_);
    tty_fd_ = -1;
  }
}

void KeyboardInput::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&KeyboardInput::read_loop, this);
}

void KeyboardInput::stop()
{
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

double KeyboardInput::monotonic_now_sec()
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

void KeyboardInput::read_loop()
{
  pollfd descriptor{};
  descriptor.fd = tty_fd_;
  descriptor.events = POLLIN;

  while (running_.load()) {
    const int result = ::poll(&descriptor, 1, 100);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (result == 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      break;
    }
    if ((descriptor.revents & POLLIN) == 0) {
      continue;
    }

    char buffer[64];
    const ssize_t bytes_read = ::read(tty_fd_, buffer, sizeof(buffer));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      break;
    }
    if (bytes_read == 0) {
      continue;
    }

    for (ssize_t index = 0; index < bytes_read; ++index) {
      const auto key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(buffer[index])));
      push_event(key);
    }
  }
}

void KeyboardInput::push_event(const char key)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_capacity_ == 0U) {
    ++overflow_count_;
    return;
  }
  if (events_.size() >= queue_capacity_) {
    events_.pop_front();
    ++overflow_count_;
  }
  events_.push_back(KeyEvent{key, monotonic_now_sec()});
}

std::vector<KeyEvent> KeyboardInput::drain()
{
  std::vector<KeyEvent> drained;
  std::lock_guard<std::mutex> lock(mutex_);
  drained.reserve(events_.size());
  while (!events_.empty()) {
    drained.push_back(std::move(events_.front()));
    events_.pop_front();
  }
  return drained;
}

std::size_t KeyboardInput::overflow_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return overflow_count_;
}

void KeyboardInput::restore_terminal()
{
  if (terminal_configured_ && tty_fd_ >= 0) {
    (void)::tcsetattr(tty_fd_, TCSANOW, &original_terminal_);
  }
  terminal_configured_ = false;
}

}  // namespace torpedo_control_v2
