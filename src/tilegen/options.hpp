#pragma once

#include <optional>
#include <string>

namespace mama::tilegen {

struct Options {
  std::string output_folder;
  std::string osm_file;
  // meters
  std::optional<uint32_t> max_precompute_path_length;

  static Options Parse(int argc, char** argv);

private:
  Options() = default;

  static void PrintUsage(const char* program_name);
  static Options ParseOrThrow(int argc, char** argv);
};

} // namespace mama::tilegen
