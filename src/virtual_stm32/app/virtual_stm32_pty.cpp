// Runs the virtual STM32 behind a pseudo-terminal, so ordinary serial code (the ROS 2 driver, a
// terminal program, pyserial) can talk to it exactly as it would to the real board.
//
//   virtual_stm32_pty [--link /tmp/vstm32] [--speed 1.0] [--slip 0.0] [--stall] [--drop 0.0]
//
// Linux/macOS only. NOTE: this front end has not been compiled or run in the development
// environment used so far (Windows); the model it wraps (VirtualStm) is fully unit-tested.
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include "virtual_stm32/virtual_stm.hpp"

int main(int argc, char** argv) {
  std::string link_path = "/tmp/vstm32";
  double speed = 1.0;
  stm::FaultConfig faults;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--stall") faults.stall = true;
    else if (i + 1 < argc && a == "--link") link_path = argv[++i];
    else if (i + 1 < argc && a == "--speed") speed = std::atof(argv[++i]);
    else if (i + 1 < argc && a == "--slip") faults.slip = std::atof(argv[++i]);
    else if (i + 1 < argc && a == "--drop") faults.drop_rate = std::atof(argv[++i]);
  }

  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || grantpt(master) < 0 || unlockpt(master) < 0) {
    std::perror("pty");
    return 1;
  }
  // Raw mode: no echo, no line editing, no CR/LF translation (the slave behaves like a UART).
  termios tio;
  if (tcgetattr(master, &tio) == 0) {
    cfmakeraw(&tio);
    tcsetattr(master, TCSANOW, &tio);
  }
  const char* slave = ptsname(master);
  unlink(link_path.c_str());
  if (symlink(slave, link_path.c_str()) != 0) std::perror("symlink");
  std::printf("virtual STM32 listening on %s -> %s\n", link_path.c_str(), slave);
  std::fflush(stdout);

  stm::VirtualStm device(mdp::Pose{20, 10, mdp::kPi / 2}, faults);
  auto last = std::chrono::steady_clock::now();
  for (;;) {
    pollfd p{master, POLLIN, 0};
    if (poll(&p, 1, 5) > 0 && (p.revents & POLLIN)) {
      char buf[256];
      const ssize_t n = read(master, buf, sizeof buf);
      if (n > 0) device.receive(std::string_view(buf, static_cast<std::size_t>(n)));
    }
    const auto now = std::chrono::steady_clock::now();
    device.step(std::chrono::duration<double>(now - last).count() * speed);
    last = now;
    const std::string out = device.take_output();
    if (!out.empty() && write(master, out.data(), out.size()) < 0) std::perror("write");
  }
}
