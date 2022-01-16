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

void resampleHDR(float *dst_ptr, float *src_ptr,
                 uint32_t src_w, uint32_t src_h,
                 uint32_t dst_w, uint32_t dst_h,
                 uint32_t num_components);

}
