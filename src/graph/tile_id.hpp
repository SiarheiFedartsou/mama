#pragma once

#include <s2/s2cell_id.h>
#include "tile_level.hpp"

namespace mama {

using TileId = uint32_t;

static inline constexpr uint64_t kTileIdBitSize = 64 - 2 * (S2CellId::kMaxLevel - graph::kTileLevel);

inline TileId S2CellIdToTileId(S2CellId cell_id) {
  static_assert(kTileIdBitSize == 26);
  assert(cell_id.level() == kTileLevel);

  // based on S2CellId::lsb_for_level
  constexpr uint64_t kShift = 64 - kTileIdBitSize;
  const auto tile_id = cell_id.id() >> kShift;
  assert(tile_id < (1 << kTileIdBitSize));
  return static_cast<TileId>(tile_id);
}

}  // namespace mama