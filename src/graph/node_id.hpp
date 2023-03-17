#pragma once

#include <cstdint>
#include "tile_id.hpp"

namespace mama {

struct NodeId {
  uint64_t value = 0;

  explicit NodeId(uint64_t value) : value(value) {}
  NodeId(TileId tile_id, uint32_t node_index)
      : value(static_cast<uint64_t>(tile_id) << (64 - kTileIdBitSize) | node_index) {}

  TileId tile_id() const { return static_cast<TileId>(value >> (64 - kTileIdBitSize)); }
  uint32_t node_index() const { return static_cast<uint32_t>(value); }
};

}  // namespace mama
