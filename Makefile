CXX = g++

CXXFLAGS = -std=c++17 -O3 -Wall -I include

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
TARGET = $(BIN_DIR)/cppwrf_core

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	@echo "Linking objects to create executable..."
	$(CXX) $(OBJECTS) -o $(TARGET)
	@echo ">>> BUILD SUCCESSFUL: $(TARGET) <<<"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJ_DIR) $(BIN_DIR) data/*.csv
	@echo "Cleaned."

run: all
	@echo "Running Simulation..."
	@./$(TARGET)
