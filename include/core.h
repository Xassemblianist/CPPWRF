#ifndef CORE_H
#define CORE_H

#include <vector>
#include <string>
#include <cmath>

struct alignas(16) Vec3 {
    float x, y, z;
    float padding;
};

struct Cell {
    float temp;
    float pressure;
    float wind_u;
    float wind_v;
    float wind_w;
};

class AtmosEngine {
private:
    int width, height, depth;
    int total_cells;
    
    Cell* grid_data; 
    
public:
    AtmosEngine(int w, int h, int d);
    ~AtmosEngine();

    void initialize_state();
    void compute_step_asm_style();
    void export_csv(const std::string& filename);
    
    inline Cell& get(int x, int y, int z) {
        return grid_data[x + width * (y + height * z)];
    }
};

#endif