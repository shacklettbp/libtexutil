#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <OpenImageIO/imageio.h>

#include "texutil.hpp"

using namespace std;
using namespace texutil;
using namespace OIIO;

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
    assert(src_channels == 3);

    vector<float> src_data(src_width * src_height * src_channels);

    src_img->read_image(TypeDesc::FLOAT, src_data.data());
    src_img->close();

    auto generateAndWriteMips = [](ofstream &file,
                                   uint32_t base_width,
                                   uint32_t base_height,
                                   uint32_t num_components,
                                   float *base_data) {
        uint32_t num_mips = log2(max(base_width, base_height)) + 1;

        uint32_t bytes_per_pixel = num_components * sizeof(float);

        file.write((char *)&num_mips, sizeof(uint32_t));
        file.write((char *)&base_width, sizeof(uint32_t));
        file.write((char *)&base_height, sizeof(uint32_t));
        file.write((char *)base_data, base_width * base_height * bytes_per_pixel);

        for (int level_idx = 1; level_idx < (int)num_mips; level_idx++) {
            uint32_t level_width = max(1u, base_width >> level_idx);
            uint32_t level_height = max(1u, base_height >> level_idx);
            vector<float> mip(level_width * level_height * num_components);

            resampleHDR(mip.data(), base_data,
                        base_width, base_height, level_width, level_height,
                        num_components);

            file.write((char *)&level_width, sizeof(uint32_t));
            file.write((char *)&level_height, sizeof(uint32_t));
            file.write((char *)mip.data(), sizeof(float) * mip.size());
        }
    };

    ofstream env_out(argv[2], ios::binary);
    generateAndWriteMips(env_out, src_width, src_height,
                         src_channels, src_data.data());


    auto rgbToLuminance = [](float r, float g, float b) {
        return 0.2126f * r + 0.7152f * g + 0.0722f * b;
    };

    auto getLuminance = [&](float u, float v) {
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
            uint32_t base = 3 * (y * src_width + x);

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
            float base_x = imp_per_sample * num_samples * x;
            float base_y = imp_per_sample * num_samples * y;

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

#if 0
    auto tmp_out = ImageOutput::create("/tmp/t.exr");
    ImageSpec tmp_spec(imp_dim, imp_dim, 1, TypeDesc::FLOAT);
    tmp_out->open("/tmp/t.exr", tmp_spec);
    tmp_out->write_image(TypeDesc::FLOAT, imp_data.data());
    tmp_out->close();
#endif

    generateAndWriteMips(env_out, imp_dim, imp_dim, 1, imp_data.data());
}
