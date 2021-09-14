#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <omp.h>

#include "encoder/basisu_resampler.h"
#include "encoder/basisu_resampler_filters.h"
#include "encoder/jpgd.h"
#include "encoder/lodepng.h"
#include "utils.h"
#include "rdo_bc_encoder.h"

using namespace std;
using namespace utils;
using namespace rdo_bc;

template <typename T> inline T cclamp(T value, T low, T high) {
    return (value < low) ? low : ((value > high) ? high : value);
}

template <typename T> inline T saturate(T value) {
    return cclamp<T>(value, 0, 1.0f);
}

static inline float encode_sRGB(float const intensity)
{
    float brightness;
    if (intensity < 0.0031308f)
        brightness = 12.92f * intensity;
    else
        brightness = 1.055f * pow(intensity, 1.0f/2.4f) - 0.055f;

    return brightness;
}

static inline float decode_sRGB(float const brightness)
{
    float intensity;

    if (brightness < .04045f)
        intensity = saturate(brightness * (1.0f/12.92f));
    else
        intensity = saturate(powf((brightness + .055f) * (1.0f/1.055f), 2.4f));

    return intensity;
}

static void checkResamplerStatus(basisu::Resampler& resampler)
{
    using namespace basisu;

    switch (resampler.status()) {
      case Resampler::Status::STATUS_OKAY:
        break;
      case Resampler::Status::STATUS_OUT_OF_MEMORY:
        throw std::runtime_error("Resampler or Resampler::put_line out of memory.");
        break;
      case Resampler::Status::STATUS_BAD_FILTER_NAME:
      {
        std::string msg("Unknown filter");
        throw std::runtime_error(msg);
        break;
      }
      case Resampler::Status::STATUS_SCAN_BUFFER_FULL:
        throw std::runtime_error("Resampler::put_line scan buffer full.");
        break;
    }
}

static void resample(uint8_t *dst_ptr, uint8_t *src_ptr, bool srgb,
                     basisu::Resampler::Boundary_Op wrapMode,
                     uint32_t src_w, uint32_t src_h,
                     uint32_t dst_w, uint32_t dst_h,
                     uint32_t num_components)
{
    using namespace basisu;

    const char *pFilter = "lanczos4";
    const float filter_scale = 1.f;

    auto dst = [&](int x, int y, int c) -> uint8_t& {
        return dst_ptr[(y * dst_w + x) * num_components + c];
    };

    auto src = [&](int x, int y, int c) -> uint8_t& {
        return src_ptr[(y * src_w + x) * num_components + c];
    };

    assert(src_w && src_h && dst_w && dst_h);

    if (max(src_w, src_h) > BASISU_RESAMPLER_MAX_DIMENSION
        || max(dst_w, dst_h) > BASISU_RESAMPLER_MAX_DIMENSION)
    {
        std::stringstream message;
        message << "Image larger than max supported size of "
                << BASISU_RESAMPLER_MAX_DIMENSION;
        throw std::runtime_error(message.str());
    }

    // TODO: Consider just using {decode,encode}_sRGB directly.
    float srgb_to_linear_table[256];
    if (srgb) {
      for (int i = 0; i < 256; ++i)
        srgb_to_linear_table[i] = decode_sRGB((float)i * (1.0f/255.0f));
    }

    const int LINEAR_TO_SRGB_TABLE_SIZE = 8192;
    uint8_t linear_to_srgb_table[LINEAR_TO_SRGB_TABLE_SIZE];

    if (srgb)
    {
        for (int i = 0; i < LINEAR_TO_SRGB_TABLE_SIZE; ++i)
          linear_to_srgb_table[i] = (uint8_t)cclamp<int>((int)(255.0f * encode_sRGB((float)i * (1.0f / (LINEAR_TO_SRGB_TABLE_SIZE - 1))) + .5f), 0, 255);
    }

    // Sadly the compiler doesn't realize that num_components is a
    // constant value for each template so size the arrays to the max.
    // number of components.
    std::vector<float> samples[4];
    Resampler *resamplers[4];

    resamplers[0] = new Resampler(src_w, src_h, dst_w, dst_h,
                                  wrapMode,
                                  0.0f, 1.0f,
                                  pFilter, nullptr, nullptr,
                                  filter_scale, filter_scale,
                                  0, 0);
    checkResamplerStatus(*resamplers[0]);
    samples[0].resize(src_w);

    for (uint32_t i = 1; i < num_components; ++i)
    {
        resamplers[i] = new Resampler(src_w, src_h, dst_w, dst_h,
                                      wrapMode,
                                      0.0f, 1.0f,
                                      pFilter,
                                      resamplers[0]->get_clist_x(),
                                      resamplers[0]->get_clist_y(),
                                      filter_scale, filter_scale,
                                      0, 0);
        checkResamplerStatus(*resamplers[i]);
        samples[i].resize(src_w);
    }
    uint32_t dst_y = 0;

    for (uint32_t src_y = 0; src_y < src_h; ++src_y)
    {
        // Put source lines into resampler(s)
        for (uint32_t x = 0; x < src_w; ++x)
        {
            for (uint32_t ci = 0; ci < num_components; ++ci)
            {
              const uint32_t v = src(x, src_y, ci);

              if (!srgb || (ci == 3))
                  samples[ci][x] = v * (1.0f / 255.0f);
              else
                  samples[ci][x] = srgb_to_linear_table[v];
            }
        }

      for (uint32_t ci = 0; ci < num_components; ++ci)
      {
          if (!resamplers[ci]->put_line(&samples[ci][0]))
          {
              checkResamplerStatus(*resamplers[ci]);
          }
      }

      // Now retrieve any output lines
      for (;;)
      {
        uint32_t ci;
        for (ci = 0; ci < num_components; ++ci)
        {
            const float *pOutput_samples = resamplers[ci]->get_line();
            if (!pOutput_samples)
                break;

            const bool linear_flag = !srgb || (ci == 3);

            for (uint32_t x = 0; x < dst_w; x++)
            {
                // TODO: Add dithering
                if (linear_flag) {
                    int j = (int)(255.0f * pOutput_samples[x] + .5f);
                    dst(x, dst_y, ci) = cclamp<int>(j, 0, 255);
                } else {
                    int j = (int)((LINEAR_TO_SRGB_TABLE_SIZE - 1) * pOutput_samples[x] + .5f);
                    dst(x, dst_y, ci) = (uint8_t)linear_to_srgb_table[cclamp<int>(j, 0, LINEAR_TO_SRGB_TABLE_SIZE - 1)];
                }
              }
          }
          if (ci < num_components)
              break;

          ++dst_y;
      }
  }

  for (uint32_t i = 0; i < num_components; ++i)
      delete resamplers[i];
}

enum class TextureType {
    BaseColor,
    MetallicRoughness,
    Normal,
};

using CompressedMip = rdo_bc::rdo_bc_encoder;

enum class BlockFormat {
    BC7,
    BC5,
};

rdo_bc::rdo_bc_params getRDODefaults() 
{
    rdo_bc::rdo_bc_params rp;
    rp.m_rdo_max_threads = std::min(std::max(1, omp_get_max_threads()), 128);
    rp.m_status_output = false;
    rp.m_use_bc7e = true;
    rp.m_rdo_lambda = 1.0f;
    rp.m_lookback_window_size = 1024;
    rp.m_custom_lookback_window_size = true;

    //rp.m_rdo_smooth_block_error_scale = 70.f;
    //rp.m_custom_rdo_smooth_block_error_scale = true;

    return rp;
}

static CompressedMip compressBC7(const image_u8 &src, bool alpha)
{
    auto rp = getRDODefaults();
    rp.m_dxgi_format = DXGI_FORMAT_BC7_UNORM;

    rdo_bc::rdo_bc_encoder encoder;
    if (!encoder.init(src, rp)) {
        cerr << "Encoder init error\n" << endl;
        abort();
    }

    if (!encoder.encode()) {
        cerr << "Encoder encode error\n" << endl;
        abort();
    }

    assert(alpha || !encoder.get_has_alpha());

    return encoder;
}

static CompressedMip compressBC5(const image_u8 &src,
                                 uint32_t first_channel,
                                 uint32_t second_channel)
{
    auto rp = getRDODefaults();
    rp.m_dxgi_format = DXGI_FORMAT_BC5_UNORM;
    // These options don't work properly with RDO
    rp.m_bc45_channel0 = 0;
    rp.m_bc45_channel1 = 1;

    image_u8 src_new(src.width(), src.height());
    for (int i = 0; i < (int)src.total_pixels(); i++) {
        const auto &src_pix = src.get_pixels()[i];
        auto &dst_pix = src_new.get_pixels()[i];
        dst_pix[0] = src_pix[first_channel];
        dst_pix[1] = src_pix[second_channel];
        dst_pix[2] = 0.f;
        dst_pix[3] = 1.f;
    }

    rdo_bc::rdo_bc_encoder encoder;
    if (!encoder.init(src_new, rp)) {
        cerr << "Encoder init error\n" << endl;
        abort();
    }

    if (!encoder.encode()) {
        cerr << "Encoder encode error\n" << endl;
        abort();
    }

    return encoder;
}

int main(int argc, char *argv[]) 
{
    if (argc < 4) {
        printf("%s TYPE SRC DST", argv[0]);
        exit(EXIT_FAILURE);
    }

    string type_str = argv[1];
    string src_str = argv[2];
    string dst_str = argv[3];

    TextureType tex_type;
    if (type_str == "base") {
        tex_type = TextureType::BaseColor;
    } else if (type_str == "mr") {
        tex_type = TextureType::MetallicRoughness;
    } else if (type_str == "normal") {
        tex_type = TextureType::Normal;
    } else {
        cerr << "Invalid texture type" << endl;
        exit(EXIT_FAILURE);
    }

    string src_ext = src_str.substr(src_str.rfind(".") + 1);

    uint8_t *src;
    uint32_t src_width, src_height;
    if (src_ext == "png") {
        if (lodepng_decode32_file(&src, &src_width, &src_height,
                                   src_str.c_str()) > 0) {
            cerr << "Failed to load png: " << src_str << endl;
            exit(EXIT_FAILURE);
        }
    } else if (src_ext == "jpg") {
        int n;
        src = jpgd::decompress_jpeg_image_from_file(
            src_str.c_str(), (int *)&src_width, (int *)&src_height, &n, 4);
        if (!src) {
            cerr << "Failed to load jpg: " << src_str << endl;
            exit(EXIT_FAILURE);
        }
    } else {
        cerr << "Unknown file type" << endl;
        exit(EXIT_FAILURE);
    }
    printf("Loaded %s (%u, %u)\n", src_ext.c_str(), src_width, src_height);

    uint8_t start[4];
    start[0] = src[0];
    start[1] = src[1];
    start[2] = src[2];
    start[3] = src[3];
    bool uniform = true;
    for (int i = 0; i < (int)(src_width * src_height); i++) {
        if (tex_type == TextureType::MetallicRoughness) {
            if (src[i * 4 + 1] != start[1] || src[i * 4 + 2] != start[2]) {
                uniform = false;
                break;
            }
        } else {
            if (src[i * 4] != start[0] || src[i * 4 + 1] != start[1] ||
                src[i * 4 + 2] != start[2] || src[i * 4 + 3] != start[3]) {
                uniform = false;
                break;
            }
        }
    }

    if (uniform) {
        printf("Uniform texture detected\n");
        src_width = 1;
        src_height = 1;
    }

    bool srgb = tex_type == TextureType::BaseColor;

    uint32_t max_dim = max(src_width, src_height);
    uint32_t num_levels = log2(max_dim) + 1;
    vector<image_u8> mips;
    mips.reserve(num_levels);
    mips.emplace_back(src_width, src_height);
    memcpy(mips.back().get_pixels().data(), src, mips.back().total_pixels() * 4);

    for (int level_idx = 1; level_idx < (int)num_levels; level_idx++) {
        uint32_t level_width = max(1u, mips[0].width() >> level_idx);
        uint32_t level_height = max(1u, mips[0].height() >> level_idx);
        image_u8 level(level_width, level_height);

        resample((uint8_t *)level.get_pixels().data(), src, srgb,
                 basisu::Resampler::Boundary_Op::BOUNDARY_CLAMP,
                 mips[0].width(), mips[0].height(), level_width, level_height, 4);
        mips.push_back(level);
    }

    for (int level_idx = 0; level_idx < (int)num_levels; level_idx++) {
        image_u8 &level = mips[level_idx];
        uint32_t level_width = level.width();
        uint32_t level_height = level.height();

        if (tex_type == TextureType::Normal) {
            for (int i = 0; i < int(level_width * level_height); i++) {
                auto &pixel = level.get_pixels()[i];
                uint8_t x_raw = pixel[0];
                uint8_t y_raw = pixel[1];
                uint8_t z_raw = pixel[2];

                float x = (float(x_raw) / 255.f) * 2.f - 1;
                float y = (float(y_raw) / 255.f) * 2.f - 1;
                float z = (float(z_raw) / 255.f) * 2.f - 1;

                float len = sqrtf(x*x + y*y + z*z);
                assert(len > 0);

                x /= len;
                y /= len;
                z /= len;

                pixel[0] = uint8_t(min(255.f, roundf(((x + 1) / 2.f) * 255.f)));
                pixel[1] = uint8_t(min(255.f, roundf(((y + 1) / 2.f) * 255.f)));
                pixel[2] = 0;
                pixel[3] = 255;
            }
        }

        if (tex_type == TextureType::MetallicRoughness) {
            for (int i = 0; i < int(level_width * level_height); i++) {
                auto &pixel = level.get_pixels()[i];
                uint8_t r = pixel[1];
                uint8_t m = pixel[2];
                pixel[0] = r;
                pixel[1] = m;
                pixel[2] = 0;
                pixel[3] = 255;
            }
        }
    }
    
    vector<pair<uint8_t *, size_t>> compressed;
    compressed.reserve(num_levels);
    for (int level_idx = 0; level_idx < (int)num_levels; level_idx++) {
        const image_u8 &lvl = mips[level_idx];

        uint8_t *data;
        size_t num_bytes;
        uint32_t r = lodepng_encode32(&data, &num_bytes, (uint8_t *)lvl.get_pixels().data(), lvl.width(), lvl.height());
        if (r) {
            cerr << "PNG compression failed" << endl;
            abort();
        }
        compressed.emplace_back(data, num_bytes);
    }

    ofstream out_file(dst_str, ios::binary | ios::out | ios::trunc);
    auto write = [&out_file](auto val) {
        out_file.write((const char *)&val, sizeof(decltype(val)));
    };

    write(uint32_t(0x50505050));
    write(uint32_t(num_levels));
    vector<uint32_t> out_offsets;
    uint32_t cur_offset = 0;
    for (int level_idx = 0; level_idx < int(num_levels); level_idx++) {
        write(uint32_t(mips[level_idx].width()));
        write(uint32_t(mips[level_idx].height()));
        write(uint32_t(cur_offset));
        write(uint32_t(compressed[level_idx].second));
        cur_offset += compressed[level_idx].second;
    }
    for (int level_idx = 0; level_idx < int(num_levels); level_idx++) {
        out_file.write((const char *)(compressed[level_idx].first), 
                       compressed[level_idx].second);
    }
}
