#pragma once

namespace texutil {

enum class TextureType : uint32_t {
    FourChannelSRGB,
    TwoChannelLinearRG,
    TwoChannelLinearRB,
    TwoChannelLinearGB,
    SingleChannelLinearR,
    SingleChannelLinearG,
    SingleChannelLinearB,
    SingleChannelLinearA,
    NormalMap,
};

void generateMips(const char *out_path, TextureType tex_type,
                  const uint8_t *data, uint64_t num_bytes);

}
