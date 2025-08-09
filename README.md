# Understanding Stonefish Underwater Simulation Code

## Overview
This document explains how the Stonefish underwater robotics simulation code works, breaking down each component and their interactions in an understandable way.

---

## 1. Project Architecture

The simulation consists of **4 main files** working together:

```
main.cpp
    ↓ (creates and runs)
UnderwaterTestApp (.h/.cpp)
    ↓ (uses for simulation)
UnderwaterTestManager (.h/.cpp)
```

### File Relationships:
- **main.cpp**: Entry point - starts everything
- **UnderwaterTestApp**: Graphics and user interface layer
- **UnderwaterTestManager**: Physics simulation and robot behavior

---

## 2. main.cpp - The Starting Point

### What it does:
```cpp
int main(int argc, const char * argv[])
{
    // 1. Configure graphics settings
    sf::RenderSettings s;
    s.windowW = 1200;           // Window width
    s.windowH = 900;            // Window height  
    s.aa = sf::RenderQuality::HIGH;      // Anti-aliasing
    s.shadows = sf::RenderQuality::HIGH; // Shadow quality
    s.ocean = sf::RenderQuality::HIGH;   // Ocean rendering
    
    // 2. Configure debug/helper settings
    sf::HelperSettings h;
    h.showFluidDynamics = false;    // Hide fluid flow visualization
    h.showCoordSys = false;         // Hide coordinate systems
    h.showBulletDebugInfo = false;  // Hide physics debug info
    
    // 3. Create simulation with 200 steps per second
    UnderwaterTestManager simulationManager(200.0);
    simulationManager.setRealtimeFactor(1.0);  // Real-time speed
    
    // 4. Create and run the application
    UnderwaterTestApp app(DATA_DIR_PATH, s, h, &simulationManager);
    app.Run();
    
    return 0;
}
```

### Key Components:
- **RenderSettings**: Controls how pretty the simulation looks
- **HelperSettings**: Controls what debug information to show
- **SimulationManager**: The brain that runs physics
- **App**: The window and user interface

---

## 3. UnderwaterTestManager - The Physics Brain

### Purpose:
This is where all the underwater physics, robots, and environment are created and managed.

### Class Structure:
```cpp
class UnderwaterTestManager : public sf::SimulationManager
{
public:
    UnderwaterTestManager(sf::Scalar stepsPerSecond);
    void BuildScenario();                    // Creates the underwater world
    void SimulationStepCompleted(sf::Scalar timeStep);  // Called every physics step
};
```

### BuildScenario() - Creating the Underwater World

This function is like building a virtual underwater world from scratch:

#### Step 1: Materials (What things are made of)
```cpp
CreateMaterial("Dummy", sf::UnitSystem::Density(sf::CGS, sf::MKS, 0.9), 0.3);
CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.9);
CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.6);
```

**What this means:**
- **Density**: How heavy the material is (affects buoyancy)
- **Friction**: How slippery/rough surfaces are when they touch
- Different materials behave differently underwater

#### Step 2: Looks (How things appear visually)
```cpp
CreateLook("yellow", sf::Color::RGB(1.f, 0.9f, 0.f), 0.3f, 0.f);
CreateLook("grey", sf::Color::RGB(0.3f, 0.3f, 0.3f), 0.4f, 0.5f);
```

**What this means:**
- **Color**: RGB values (Red, Green, Blue)
- **Roughness**: How shiny/matte the surface is
- **Metallic**: How metallic it looks

#### Step 3: Environment Setup
```cpp
EnableOcean(0.0);  // Create ocean at surface level
getOcean()->setWaterType(0.2);  // Water clarity/turbidity
getOcean()->AddVelocityField(new sf::Jet(...));     // Water currents
getOcean()->AddVelocityField(new sf::Uniform(...)); // Ocean currents
getAtmosphere()->SetSunPosition(0.0, 60.0);        // Sun angle for lighting
```

**What this creates:**
- **Ocean**: Water with realistic physics (buoyancy, drag, currents)
- **Water currents**: Moving water that affects the robot
- **Lighting**: Realistic underwater lighting from sun

#### Step 4: Seabed and Obstacles
```cpp
sf::Terrain* seabed = new sf::Terrain("Seabed", sf::GetDataPath() + "terrain.png", 
                                      1.0, 1.0, 5.0, "Rock", "seabed", 5.f);
AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,15.0)));
```

**What this creates:**
- **Terrain**: 3D seabed from a height map image
- **Static objects**: Things that don't move (rocks, structures)

#### Step 5: Building the Robot (GIRONA500 AUV)

##### Hull Construction:
```cpp
sf::Polyhedron* hullB = new sf::Polyhedron("HullBottom", phy, 
                                          sf::GetDataPath() + "hull_hydro.obj", 
                                          sf::Scalar(1), sf::I4(), 
                                          "Fiberglass", "yellow", sf::Scalar(0.003));
```

**What this means:**
- **Polyhedron**: Complex 3D shape loaded from file
- **hull_hydro.obj**: 3D mesh file with hydrodynamic properties
- **Fiberglass material**: Affects weight and physics
- **0.003**: Hydrodynamic coefficient for water resistance

##### Compound Assembly:
```cpp
sf::Compound* vehicle = new sf::Compound("Vehicle", phy, hullB, sf::I4());
vehicle->AddExternalPart(hullP, sf::Transform(...));  // Add port hull
vehicle->AddExternalPart(hullS, sf::Transform(...));  // Add starboard hull
vehicle->AddInternalPart(batteryCyl, sf::Transform(...)); // Add battery
```

**What this creates:**
- **Compound body**: Multiple parts acting as one rigid object
- **External parts**: Visible components that interact with water
- **Internal parts**: Hidden components that add mass/inertia

#### Step 6: Robotic Arm (Manipulator)
```cpp
std::vector<sf::SolidEntity*> arm;
arm.push_back(baseLink);
arm.push_back(link1);
arm.push_back(link2);
// ... more links

auv->DefineLinks(vehicle, arm);
auv->DefineRevoluteJoint("Joint1", "ArmBaseLink", "ArmLink1", ...);
```

**What this creates:**
- **Kinematic chain**: Connected links that can move relative to each other
- **Joints**: Connection points between links with motion constraints
- **6-DOF arm**: 6 degrees of freedom for complex manipulation

#### Step 7: Actuators (Things that make the robot move)

##### Thrusters:
```cpp
sf::Thruster* thruster = new sf::Thruster(name, propeller, rotorDynamics, 
                                          thrustModel, diameter, righthanded, 
                                          maxRPM, inverted, realistic);
```

**What each thruster includes:**
- **Propeller mesh**: 3D model of spinning propeller
- **Rotor dynamics**: How the motor accelerates/decelerates
- **Thrust model**: Converts RPM to force (realistic physics)
- **Position**: Where on the vehicle it's mounted

##### Servo Motors:
```cpp
sf::Servo* srv1 = new sf::Servo("Servo1", 1.0, 1.0, 100.0);
auv->AddJointActuator(srv1, "Joint1");
```

**What this does:**
- **Controls joint angles**: Makes arm links move
- **PID control**: Automatically reaches desired positions
- **Torque limits**: Realistic motor limitations

#### Step 8: Sensors (Robot's senses)

##### Navigation Sensors:
```cpp
sf::Odometry* odom = new sf::Odometry("Odom");        // Position/velocity
sf::IMU* imu = new sf::IMU("IMU");                    // Acceleration/rotation
sf::DVL* dvl = new sf::DVL("DVL", 30.0, false);      // Velocity relative to seabed
sf::Pressure* press = new sf::Pressure("Pressure");   // Depth measurement
```

##### Vision Sensors:
```cpp
sf::FLS* fls = new sf::FLS("FLS", 256, 500, 150.0, 30.0, 1.0, 20.0, sf::ColorMap::GREEN_BLUE);
sf::SSS* sss = new sf::SSS("SSS", 800, 400, 70.0, 1.5, 50.0, 1.0, 100.0, sf::ColorMap::GREEN_BLUE);
```

**What these do:**
- **FLS (Forward-Looking Sonar)**: "Eyes" that see ahead using sound
- **SSS (Side-Scan Sonar)**: Creates images of the seabed
- **MSIS (Multi-beam Imaging Sonar)**: 3D sonar imaging

### SimulationStepCompleted() - The Update Loop

This function runs **200 times per second** (based on our settings):

```cpp
void UnderwaterTestManager::SimulationStepCompleted(sf::Scalar timeStep)
{
    // This is where you can:
    // - Read sensor data
    // - Make control decisions  
    // - Send commands to actuators
    // - Log data
}
```

---

## 4. UnderwaterTestApp - The Visual Interface

### Purpose:
This handles everything you see and interact with on screen.

### Class Structure:
```cpp
class UnderwaterTestApp : public sf::GraphicalSimulationApp
{
public:
    UnderwaterTestApp(...);
    void DoHUD();           // Creates on-screen interface
    void InitializeGUI();   // Sets up text rendering
};
```

### InitializeGUI() - Setting up text display:
```cpp
void UnderwaterTestApp::InitializeGUI()
{
    largePrint = new sf::OpenGLPrinter(sf::GetShaderPath() + STANDARD_FONT_NAME, 64.0);
}
```

### DoHUD() - Interactive Controls

This is where all the commented-out code would create:

#### Manual Controls:
```cpp
// Example of what the commented code does:
sf::Servo* srv1 = getSimulationManager()->getActuator("GIRONA500/Servo1");
sf::Scalar sp = getGUI()->DoSlider(id, 180.f, 10.f, 150.f, -1.0, 1.0, 
                                   srv1->getPosition(), "Servo1");
srv1->setDesiredPosition(sp);
```

**What this creates:**
- **Slider on screen**: You can drag to control servo position
- **Real-time feedback**: Shows current servo position
- **Manual override**: Take control from automatic systems

#### Data Visualization:
```cpp
// Example of plotting sensor data:
sf::ScalarSensor* dvl = getSimulationManager()->getSensor("GIRONA500/dvl");
getGUI()->DoTimePlot(id, 200, 20, 400, 200, dvl, dims, "Water velocity");
```

**What this shows:**
- **Real-time graphs**: Sensor data plotted over time
- **Multiple channels**: Different sensor readings on same plot
- **Visual debugging**: See what the robot is sensing

---

## 5. How Everything Works Together

### The Complete Flow:

1. **main.cpp** starts everything:
   - Sets up graphics quality
   - Creates the simulation manager
   - Creates the app window
   - Starts the main loop

2. **UnderwaterTestManager** creates the world:
   - Builds underwater environment
   - Creates the robot with all its parts
   - Sets up physics simulation
   - Defines materials and their interactions

3. **Simulation Loop** runs continuously:
   - Physics engine updates robot position
   - Water affects robot with buoyancy/drag/currents  
   - Sensors measure the environment
   - Actuators respond to control commands
   - Graphics render the scene

4. **UnderwaterTestApp** handles interaction:
   - Displays the 3D underwater scene
   - Shows sensor data and controls
   - Allows manual override of robot systems
   - Provides debugging information

### Data Flow:
```
Environment → Sensors → Control Logic → Actuators → Robot Movement → Environment
     ↑                                                                      ↓
     ←←←←←←←←←←←←←← Physics Simulation ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
```

---

## 6. Key Concepts Explained

### Physics Integration:
- **Stonefish uses Bullet Physics**: Industry-standard physics engine
- **Featherstone algorithm**: Efficient multi-body dynamics for robots
- **Hydrodynamics**: Realistic water effects on moving objects

### Coordinate Systems:
- **NED (North-East-Down)**: Navigation coordinate system
- **Local frames**: Each robot part has its own coordinate system
- **Transforms**: Convert between different coordinate systems

### Real-time Simulation:
- **200 Hz physics**: Very smooth and accurate simulation
- **Real-time factor 1.0**: Simulation runs at real-world speed
- **Graphics rendering**: Separate from physics for smooth visuals

### Modular Design:
- **Separation of concerns**: Physics, graphics, and control are separate
- **Easy modification**: Can change robot design without affecting other parts
- **Reusable components**: Sensors and actuators work on any robot

---

## 7. Common Use Cases

### Research Applications:
- **Algorithm testing**: Test navigation and control algorithms
- **Sensor fusion**: Combine multiple sensor readings
- **Mission planning**: Plan underwater missions safely
- **Hardware validation**: Test before building real robot

### Development Workflow:
1. **Design robot** in CAD software
2. **Export meshes** for visual and physics models  
3. **Configure simulation** with materials and sensors
4. **Test algorithms** in safe virtual environment
5. **Deploy to real robot** with confidence

This simulation framework provides a complete underwater robotics development environment that bridges the gap between theoretical research and real-world deployment.
