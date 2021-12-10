#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

#include "texutil.hpp"

using namespace std;
using namespace texutil;

int main(int argc, char *argv[])
{
    if (argc < 4) {
        fprintf(stderr, "%s SRC DST TYPE\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    TextureType type;
    if (!strcmp(argv[3], "FourChannelSRGB")) {
        type = TextureType::FourChannelSRGB;
    } else if (!strcmp(argv[3], "TwoChannelLinear")) {
        type = TextureType::TwoChannelLinear;
    } else if (!strcmp(argv[3], "NormalMap")) {
        type = TextureType::NormalMap;
    }

    ifstream src_file(argv[1], ios::binary | ios::ate);
    if (!src_file.is_open()) {
        fprintf(stderr, "Failed to open %s\n", argv[1]);
        exit(EXIT_FAILURE);
    }
    int size = src_file.tellg();
    src_file.seekg(0, ios::beg);

    vector<uint8_t> data(size);
    src_file.read((char *)data.data(), size);

    generateMips(argv[2], type, data.data(), data.size());
}
