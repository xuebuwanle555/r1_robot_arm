#include <iostream>
#include <string>

#include "test_framework.hpp"

int main(int argc, char** argv) {
  std::string filter;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--list") {
      for (const auto& test : r1_test::registry()) {
        std::cout << test.name << "\n";
      }
      return 0;
    }
    if (arg == "--filter" && i + 1 < argc) {
      filter = argv[++i];
      continue;
    }
    if (arg.rfind("--filter=", 0) == 0) {
      filter = arg.substr(9);
      continue;
    }
    filter = arg;
  }

  std::cout << "r1_robot unit tests (" << r1_test::registry().size()
            << " cases)\n\n";
  return r1_test::runAll(filter);
}
