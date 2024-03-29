#include "options.hpp"

#include <iostream>
#include <optional>
#include <string>

namespace mama::tilegen {

// static
Options Options::Parse(int argc, char** argv) {
  Options options;
  try {
    // -1 to skip the program name
    options = ParseOrThrow(argc - 1, argv + 1);
  } catch (const std::runtime_error& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    std::exit(1);
  }
  return options;
}

// static
void Options::PrintUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " [options] <osm_file> <output_folder>\n";
  std::cerr << "\n";
  std::cerr << "Options:\n";
  std::cerr << "  --max-precompute-path-length <value>    Maximum precompute path length\n";
}

// static
Options Options::ParseOrThrow(int argc, char** argv) {
  Options options;
  for (int arg_index = 0; arg_index < argc; ++arg_index) {
    std::string arg = argv[arg_index];
    if (arg == "--max-precompute-path-length") {
      if (arg_index + 1 >= argc) {
        throw std::runtime_error("Missing value for --max-precompute-path-length");
      }
      options.max_precompute_path_length = std::stoi(argv[arg_index + 1]);
      ++arg_index;
    } else {
      if (options.osm_file.empty()) {
        options.osm_file = arg;
      } else if (options.output_folder.empty()) {
        options.output_folder = arg;
      } else {
        throw std::runtime_error("Unexpected argument: " + arg);
      }
    }
  }
  return options;
}

}  // namespace mama::tilegen
