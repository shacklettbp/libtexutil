#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <OpenImageIO/imageio.h>

#include "texutil.hpp"

using namespace std;
using namespace texutil;
using namespace OIIO;

inline uint64_t alignOffset(uint64_t offset, uint64_t alignment)
{
    return ((offset + alignment - 1) / alignment) * alignment;
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr, "%s SRC DST\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    auto src_img = ImageInput::open(argv[1]);
    if (!src_img) {
        cerr << "Failed to open: " << argv[1] << endl;
        abort();
    }

    const auto &src_spec = src_img->spec();

    uint32_t src_width = src_spec.width;
    uint32_t src_height = src_spec.height;
    uint32_t src_channels = src_spec.nchannels;

    vector<float> src_stage(src_width * src_height * src_channels);
    vector<float> src_data(src_width * src_height * 4, 1.f);

    src_img->read_image(TypeDesc::FLOAT, src_stage.data());
    src_img->close();

    for (int y = 0; y < (int)src_height; y++) {
        for (int x = 0; x < (int)src_width; x++) {
            for (int c = 0; c < (int)src_channels; c++) {
                src_data[4 * (y * src_width + x) + c] =
                    src_stage[src_channels * (y * src_width + x) + c];
            }
        }
    }

    auto generateAndWriteMips = [](ofstream &file,
                                   uint32_t base_width,
                                   uint32_t base_height,
                                   uint32_t num_components,
                                   float *base_data) {
        const uint32_t level_alignment = 16;
        uint32_t num_mips = log2(max(base_width, base_height)) + 1;

        uint32_t bytes_per_pixel = num_components * sizeof(float);
        uint64_t top_bytes = base_width * base_height * bytes_per_pixel;

        file.write((char *)&num_mips, sizeof(uint32_t));
        file.write((char *)&base_width, sizeof(uint32_t));
        file.write((char *)&base_height, sizeof(uint32_t));

        vector<vector<float>> mips;
        mips.reserve(num_mips - 1);

        uint64_t cur_offset = alignOffset(top_bytes, level_alignment);

        for (int level_idx = 1; level_idx < (int)num_mips; level_idx++) {
            uint32_t level_width = max(1u, base_width >> level_idx);
            uint32_t level_height = max(1u, base_height >> level_idx);
            mips.emplace_back(level_width * level_height * num_components);

            resampleHDR(mips.back().data(), base_data,
                        base_width, base_height, level_width, level_height,
                        num_components);

            cur_offset += alignOffset(mips.back().size() * sizeof(float),
                                      level_alignment);
        }

        auto write_pad = [&](uint32_t pad_bytes) {
            for (int i = 0; i < (int)pad_bytes; i++) {
                file.put(0);
            }
        };

        file.write((char *)&cur_offset, sizeof(uint64_t));
        auto data_start = file.tellp();
        file.write((char *)base_data, top_bytes);

        uint64_t top_rounded = alignOffset(top_bytes, level_alignment);
        write_pad(top_rounded - top_bytes);

        for (const auto &cur_mip : mips) {
            uint64_t level_bytes = sizeof(float) * cur_mip.size();
            file.write((char *)cur_mip.data(), level_bytes);

            uint64_t align_bytes = alignOffset(level_bytes, level_alignment);
            write_pad(align_bytes - level_bytes);
        }

        auto data_end = file.tellp();
        assert(data_end - data_start == (int64_t)cur_offset);
    };

    ofstream env_out(argv[2], ios::binary);
    generateAndWriteMips(env_out, src_width, src_height,
                         4, src_data.data());


    auto rgbToLuminance = [](float r, float g, float b) {
        return 0.2126f * r + 0.7152f * g + 0.0722f * b;
    };

    auto octSphereMap = [](float u, float v) {
        auto sign = [](float x) {
            return x > 0.f ? 1.f : (
                x == 0.f ? 0.f : (
                    -1.f
                )
            );
        };

        u = u * 2.f - 1.f;
        v = v * 2.f - 1.f;
    
        // Compute radius r (branchless)
        float d = 1.f - (fabsf(u) + fabsf(v));
        float r = 1.f - fabsf(d);
    
        // Compute phi in the first quadrant (branchless, except for the
        // division-by-zero test), using sign(u) to map the result to the
        // correct quadrant below
        float phi = (r == 0.f) ? 0.f :
            M_PI_4 * ((fabsf(v) - fabsf(u)) / r + 1.f);
    
        float f = r * sqrtf(2.f - r * r);
        float x = f * sign(u) * cosf(phi);
        float y = f * sign(v) * sinf(phi);
        float z = sign(d) * (1.f - r * r);
    
        return tuple {
            x, y, z,
        };
    };

    auto dirToLatLong = [](auto dir) {
        float dir_len = sqrtf(
            get<0>(dir) * get<0>(dir) +
            get<1>(dir) * get<1>(dir) +
            get<2>(dir) * get<2>(dir));

        float nx = get<0>(dir) / dir_len;
        float ny = get<1>(dir) / dir_len;
        float nz = get<2>(dir) / dir_len;

        return pair {
            atan2f(nx, -nz) * (M_1_PI / 2.f) + 0.5f,
            acosf(ny) * M_1_PI,
        };
    };

    auto getLuminance = [&](float imp_u, float imp_v) {
        auto env_dir = octSphereMap(imp_u, imp_v);

        auto [u, v] = dirToLatLong(env_dir);

        float pos_x = u * src_width - 0.5f;
        float pos_y = v * src_height - 0.5f;

        uint32_t left_x = pos_x;
        uint32_t right_x = left_x + 1;
        uint32_t down_y = pos_y;
        uint32_t up_y = down_y + 1;

        float offset_x = pos_x - left_x;
        float offset_y = pos_y - down_y;

        left_x = std::clamp(left_x, 0u, src_width - 1); 
        right_x = std::clamp(right_x, 0u, src_width - 1); 

        down_y = std::clamp(down_y, 0u, src_height - 1); 
        up_y = std::clamp(up_y, 0u, src_height - 1); 

        auto getRGB = [&](uint32_t x, uint32_t y) {
            uint32_t base = 4 * (y * src_width + x);

            return array<float, 3> {
                src_data[base],
                src_data[base + 1],
                src_data[base + 2],
            };
        };

        array<float, 3> vals[4];
        vals[0] = getRGB(left_x, down_y);
        vals[1] = getRGB(right_x, down_y);
        vals[2] = getRGB(left_x, up_y);
        vals[3] = getRGB(right_x, up_y);

        array<float, 3> interp;

        for (int i = 0; i < 3; i++) {
            float bottom = (1.f - offset_x) * vals[0][i] +
                offset_x * vals[1][i];
            float top = (1.f - offset_x) * vals[2][i] +
                offset_x * vals[3][i];

            interp[i] = (1.f - offset_y) * bottom + offset_y * top;
        }

        return rgbToLuminance(interp[0], interp[1], interp[2]);
    };

    uint32_t imp_width = 1u << (32u - __builtin_clz(src_width - 1));
    uint32_t imp_height = 1u << (32u - __builtin_clz(src_width - 1));

    uint32_t imp_dim = max(imp_width, imp_height);

    vector<float> imp_data(imp_dim * imp_dim);

    int num_samples = 4;
    float imp_per_sample = 1.f / (float(imp_dim) * float(num_samples));
    for (int y = 0; y < (int)imp_dim; y++) {
        for (int x = 0; x < (int)imp_dim; x++) {
            float base_x = imp_per_sample * ((float)num_samples * x + 0.5f);
            float base_y = imp_per_sample * ((float)num_samples * y + 0.5f);

            float avg_luminance = 0;
            for (int i = 0; i < num_samples; i++) {
                for (int j = 0; j < num_samples; j++) {
                    float u = base_x + i * imp_per_sample;
                    float v = base_y + j * imp_per_sample;

                    float luminance = getLuminance(u, v);

                    avg_luminance += luminance / (num_samples * num_samples);
                }
            }

            imp_data[y * imp_dim + x] = avg_luminance;
        }
    }

    auto tmp_out = ImageOutput::create("/tmp/t.exr");
    ImageSpec tmp_spec(imp_dim, imp_dim, 1, TypeDesc::FLOAT);
    tmp_out->open("/tmp/t.exr", tmp_spec);
    tmp_out->write_image(TypeDesc::FLOAT, imp_data.data());
    tmp_out->close();

    auto tmp2_out = ImageOutput::create("/tmp/t2.exr");
    ImageSpec tmp2_spec(src_width, src_height, 4, TypeDesc::FLOAT);
    tmp2_out->open("/tmp/t2.exr", tmp2_spec);
    tmp2_out->write_image(TypeDesc::FLOAT, src_data.data());
    tmp2_out->close();

    generateAndWriteMips(env_out, imp_dim, imp_dim, 1, imp_data.data());
}
