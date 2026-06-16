#include "base/logging.h"


namespace Ramulator {

Logger_t Logging::create_logger(std::string name, std::string pattern) {
  // AMMU fork edit (get-or-create): when libramulator.so is loaded as a dependency of a Python
  // extension (the AMMU pybind shim), the inline-static base-logger initializer in logging.h runs
  // once per loaded shared object against the shared spdlog registry, so a logger name (e.g.
  // "Ramulator::Base") can be registered twice. spdlog::stdout_color_st throws on duplicate —
  // return the existing logger instead. Idempotent; harmless for the standalone executable.
  if (auto existing = spdlog::get("Ramulator::" + name)) {
    return existing;
  }
  auto logger = spdlog::stdout_color_st("Ramulator::" + name);

  if (!logger) {
    throw InitializationError("Error creating logger {}!", name);
  }

  logger->set_pattern(pattern);
  logger->set_level(spdlog::level::debug);
  return logger;
}

Logger_t Logging::get(std::string name) {
  auto logger = spdlog::get("Ramulator::" + name);
  if (logger) {
    return logger;
  } else {
    throw std::runtime_error(
      fmt::format(
        "Logger {} does not exist!",
        name
      )
    );
  }
}

bool Logging::_create_base_logger() {
  auto logger = create_logger("Base");
  if (logger) {
    return true;
  } else {
    throw InitializationError("Error creating the base logger!");
  }
  return false;
}

}        // namespace Ramulator
