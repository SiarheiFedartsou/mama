#include "mama.hpp"

int main(int argc, char **argv) {
  mama::Graph graph(argv[1]);
  std::cerr << graph.Project(mama::Coordinate{13.388860, 52.517037}, 100).size()
            << std::endl;

  return 0;
}