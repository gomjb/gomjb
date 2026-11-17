#pragma once
// Production stages of a block, shared by scheduling and utility estimation.

namespace hullcad {

enum class Stage { Cutting, Welding, Heating, Painting, Handling, Erection, Testing, Count };
constexpr int kStageCount = static_cast<int>(Stage::Count);

const char* stage_key(Stage s);    // "cutting", ...
const char* stage_label(Stage s);  // Korean label

}  // namespace hullcad
