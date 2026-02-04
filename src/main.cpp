#include <iostream>
#include "../include/core.h"

int main() {
    std::cout << ">>> CPPWRF KERNEL LOADING <<<\n";
    
    AtmosEngine* engine = new AtmosEngine(100, 100, 50);
    
    engine->initialize_state();
    
    std::cout << ">>> CORE LOOP START <<<\n";
    
    // 100 adim islet
    for(int t = 0; t < 100; t++) {
        engine->compute_step_asm_style();
        
        if(t % 20 == 0) 
            std::cout << "Tick: " << t << " [OK]\n";
    }
    
    engine->export_csv("result.csv");
    
    delete engine;
    std::cout << ">>> SHUTDOWN SEQUENCE COMPLETE <<<\n";
    return 0;
}