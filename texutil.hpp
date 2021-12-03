#pragma once

namespace texutil {

enum class TextureType : uint32_t {
    FourChannelSRGB,
    TwoChannelLinear,
    NormalMap,
};

void generateMips(const char *out_path, TextureType tex_type,
                  const uint8_t *data, uint64_t num_bytes,
                  void *cb_data);

}
