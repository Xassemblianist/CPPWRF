#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <random>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <memory>
#include <algorithm>

namespace Engine {

    namespace Math {
        constexpr double PI = 3.14159265359;
        constexpr double DEG_TO_RAD = PI / 180.0;

        struct Vector3 {
            double x, y, z;

            Vector3() : x(0), y(0), z(0) {}
            Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

            Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
            Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
            Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
            Vector3 operator/(double s) const { return (s != 0) ? Vector3(x / s, y / s, z / s) : Vector3(); }

            double magnitude() const { return std::sqrt(x * x + y * y + z * z); }

            Vector3 normalize() const {
                double mag = magnitude();
                return (mag > 0) ? *this / mag : Vector3(0, 0, 0);
            }

            double dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }

            Vector3 cross(const Vector3& v) const {
                return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
            }
        };

        class Matrix3x3 {
        public:
            double m[3][3];

            Matrix3x3() {
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        m[i][j] = (i == j) ? 1.0 : 0.0;
            }

            Vector3 transform(const Vector3& v) const {
                return Vector3(
                    m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
                );
            }
        };
    }

    namespace Physics {
        constexpr double G = 9.81;
        constexpr double R_DRY = 287.05;
        constexpr double CP = 1005.0;
        constexpr double P0 = 100000.0;

        struct State {
            double temperature;
            double pressure;
            double humidity;
            double density;
            Math::Vector3 windVelocity;

            State() : temperature(288.15), pressure(101325.0), humidity(0.5), density(1.225) {}
        };

        double calculatePotentialTemp(double T, double P) {
            return T * std::pow(P0 / P, R_DRY / CP);
        }

        double calculateDensity(double P, double T) {
            return P / (R_DRY * T);
        }
    }

    namespace Core {
        class Logger {
            std::mutex logMutex;
        public:
            static Logger& getInstance() {
                static Logger instance;
                return instance;
            }

            void log(const std::string& msg, int level = 0) {
                std::lock_guard<std::mutex> lock(logMutex);
                auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

                std::tm* tm_now = std::localtime(&now);

                std::cout << "[" << std::put_time(tm_now, "%H:%M:%S") << "] ";
                if (level == 0) std::cout << "[INFO] ";
                else if (level == 1) std::cout << "[WARN] ";
                else std::cout << "[ERR] ";
                std::cout << msg << std::endl;
            }
        };

        class GridCell {
        public:
            Math::Vector3 position;
            Physics::State currentState;
            Physics::State nextState;

            GridCell(Math::Vector3 pos) : position(pos) {}

            void computeDynamics(double dt) {
                double buoyancy = (currentState.density - 1.225) * Physics::G;
                Math::Vector3 forcing(0, buoyancy, 0);

                Math::Vector3 acceleration = forcing * (1.0 / currentState.density);
                nextState.windVelocity = currentState.windVelocity + acceleration * dt;

                nextState.temperature = currentState.temperature - (0.0065 * dt);
                nextState.pressure = currentState.pressure * std::exp(-Physics::G * dt / (Physics::R_DRY * currentState.temperature));
            }

            void update() {
                currentState = nextState;
            }
        };

        class AtmosphereModel {
        private:
            std::vector<GridCell> grid;
            int dimX, dimY, dimZ;
            double resolution;
            double currentTime;

        public:
            AtmosphereModel(int x, int y, int z, double res)
                : dimX(x), dimY(y), dimZ(z), resolution(res), currentTime(0.0) {
                initializeGrid();
            }

            void initializeGrid() {
                Logger::getInstance().log("Initializing Atmospheric Grid...");
                grid.reserve(dimX * dimY * dimZ);

                for (int i = 0; i < dimX; ++i) {
                    for (int j = 0; j < dimY; ++j) {
                        for (int k = 0; k < dimZ; ++k) {
                            Math::Vector3 pos(i * resolution, j * resolution, k * resolution);
                            GridCell cell(pos);

                            double h = pos.y;
                            cell.currentState.pressure = 101325.0 * std::pow(1 - 2.25577e-5 * h, 5.25588);
                            cell.currentState.temperature = 288.15 - 0.0065 * h;
                            cell.currentState.density = Physics::calculateDensity(cell.currentState.pressure, cell.currentState.temperature);

                            grid.push_back(cell);
                        }
                    }
                }
                Logger::getInstance().log("Grid Initialization Complete. Total Cells: " + std::to_string(grid.size()));
            }

            void solveAdvection(double dt) {
                for (auto& cell : grid) {
                    cell.computeDynamics(dt);
                }
            }

            void step(double dt) {
                solveAdvection(dt);

                for (auto& cell : grid) {
                    cell.update();
                }
                currentTime += dt;
            }

            void exportData(const std::string& filename) {
                std::ofstream file(filename);
                if (file.is_open()) {
                    file << "X,Y,Z,Temp,Press,WindX,WindY,WindZ\n";
                    int limit = 0;
                    for (const auto& cell : grid) {
                        if (limit++ > 1000) break;
                        file << cell.position.x << "," << cell.position.y << "," << cell.position.z << ","
                            << cell.currentState.temperature << ","
                            << cell.currentState.pressure << ","
                            << cell.currentState.windVelocity.x << ","
                            << cell.currentState.windVelocity.y << ","
                            << cell.currentState.windVelocity.z << "\n";
                    }
                    file.close();
                }
            }

            double getTime() const { return currentTime; }
        };
    }
}

int main() {
    Engine::Core::Logger::getInstance().log("Starting WRF-Lite Core Engine...");

    Engine::Core::AtmosphereModel model(20, 50, 20, 100.0);

    const double dt = 0.1;
    const int totalSteps = 250;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < totalSteps; ++i) {
        model.step(dt);

        if (i % 25 == 0) {
            std::string msg = "Simulation Step: " + std::to_string(i) +
                " | Sim Time: " + std::to_string(model.getTime()) + "s";
            Engine::Core::Logger::getInstance().log(msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    Engine::Core::Logger::getInstance().log("Simulation Finished in " + std::to_string(diff.count()) + "s");
    Engine::Core::Logger::getInstance().log("Exporting dataset to csv...");

    model.exportData("atmos_output.csv");

    return 0;
}
