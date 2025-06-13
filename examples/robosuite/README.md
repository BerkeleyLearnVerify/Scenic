# Scenic-RoboSuite Integration Examples

This directory contains working examples of Scenic scenarios for RoboSuite robotic manipulation.

## 🚀 Quick Start

### Prerequisites
```bash
pip install robosuite scenic
```

### Run Examples

```bash
# Run the complete test suite
python examples/robosuite/test_runner.py

# Run individual scenarios
python -c "
import scenic
from scenic.simulators.robosuite import RobosuiteSimulator
import time

scenario = scenic.scenarioFromFile('examples/robosuite/basic_shapes.scenic')
scene, _ = scenario.generate()
sim = RobosuiteSimulator(use_table=False, has_renderer=True)
simulation = sim.createSimulation(scene)

for i in range(400):
    simulation.step()
    time.sleep(0.02)
    
simulation.destroy()
"
```

## 📋 Available Examples

### 1. Basic Shapes (`basic_shapes.scenic`)
- **Description**: Different geometric shapes (cube, ball, cylinder)
- **Objects**: RedCube, GreenBall, BlueCylinder
- **Purpose**: Demonstrates shape detection and rendering

### 2. Grid Formation (`grid_formation.scenic`) 
- **Description**: 3x3 grid of colored cubes
- **Objects**: 9 cubes in different colors
- **Purpose**: Tests object positioning and collision avoidance

### 3. Height Levels (`height_levels.scenic`)
- **Description**: Objects at different Z-coordinates
- **Objects**: Cubes at ground, low, medium, and high levels
- **Purpose**: Validates 3D positioning and physics

### 4. Circular Formation (`circular_formation.scenic`)
- **Description**: Objects arranged in a circle
- **Objects**: 6 cubes around perimeter + 1 center cube
- **Purpose**: Tests mathematical positioning and spatial relationships

### 5. Stacking Tower (`stacking_tower.scenic`)
- **Description**: Tower of cubes that fall and stack
- **Objects**: 6 cubes at increasing heights
- **Purpose**: Demonstrates physics simulation and object interactions

## 🔧 Configuration

All examples use the default configuration:
- **Simulator**: Custom world mode (no table)
- **Renderer**: MuJoCo viewer enabled
- **Physics**: Real-time stepping
- **Objects**: Proper shape detection (cube/ball/cylinder)

## 🧪 Testing

Run the automated test suite:
```bash
cd examples/robosuite/
python test_runner.py
```

## 🤝 Contributing

To add new examples:
1. Create `.scenic` file in `examples/robosuite/`
2. Add test case to `test_runner.py`
3. Update this README

## 📄 File Structure

```
examples/robosuite/
├── README.md                    # This file
├── test_runner.py              # Automated test suite
├── basic_shapes.scenic         # Basic geometric shapes
├── grid_formation.scenic       # 3x3 grid layout
├── height_levels.scenic        # Different Z-coordinates
├── circular_formation.scenic   # Circular arrangement
└── stacking_tower.scenic       # Physics stacking demo
```
