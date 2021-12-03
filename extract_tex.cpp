#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

using namespace std;

int main(int argc, char *argv[]) {
    ifstream tex_file(argv[1], ios::in | ios::binary);

    auto read_uint = [&tex_file]() {
        uint32_t v;
        tex_file.read((char *)&v, sizeof(uint32_t));
        return v;
    };

    auto magic = read_uint();
    if (magic != 0x50505050) {
        cerr << "Invalid texture file" << endl;
        abort();
    }
    uint32_t num_levels = read_uint();
    cout << num_levels << endl;

    vector<pair<uint32_t, uint32_t>> offsets;
    for (int i = 0; i < num_levels; i++) {
        read_uint();
        read_uint();
        uint32_t offset = read_uint();
        uint32_t num_bytes = read_uint();
        offsets.emplace_back(offset, num_bytes);
    }
    for (int i = 0; i < num_levels; i++) {
        auto [offset, num_bytes] = offsets[i];
        vector<char> data(num_bytes);
        tex_file.read(data.data(), num_bytes);
        cout << i << ": " << offset << " " <<  num_bytes << endl;

        ofstream out("/tmp/t_" + to_string(i) + ".png", ios::out | ios::binary);
        out.write(data.data(), num_bytes);
        out.close();
    }
}
