#pragma once

#include "tile_id.hpp"

namespace mama {
struct EdgeId {
  TileId tile_id = 0;
  uint32_t edge_index = 0;

  bool operator==(const EdgeId& other) const { return tile_id == other.tile_id && edge_index == other.edge_index; }

  template <typename H>
  friend H AbslHashValue(H h, const EdgeId& e) {
    return H::combine(std::move(h), e.tile_id, e.edge_index);
  }
};
}  // namespace mama