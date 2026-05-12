#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "rlImGui.h"
#include <vector>
#include <string>
#include <cmath>

// ============================================================================
// --- PHYSICS DATA STRUCTURES ---
// ============================================================================

// Represents the state of our orbiting baseball-sized test mass.
struct State
{
    Vector3 pos; // Position vector (x, y, z) in 3D space
    Vector3 vel; // Velocity vector (vx, vy, vz)
};

// Holds all the adjustable variables for our simulation environment.
// These are directly tied to the ImGui sliders.
struct SimulationParams
{
    float primaryMass = 1000.0f; // The mass of the central object (Sun or Black Hole)
    float warpDepth = 50.0f;     // The baseline visual depth of the gravity well
    float speedOfLight = 100.0f; // Scaled down speed of light to make relativistic effects visible quickly
    float gridElasticity = 1.0f; // How 'stiff' the visual spacetime grid looks
    bool blackHoleMode = false;  // Toggles rendering of the accretion disk and event horizon
    float timeStep = 0.016f;     // Physics simulation step size (roughly 60 FPS)
};

// ============================================================================
// --- NUMERICAL INTEGRATOR (PHYSICS ENGINE) ---
// ============================================================================

class GRPhysicsEngine
{
public:
    // Calculates the acceleration vector at any given point in space
    // using a Post-Newtonian approximation of the Schwarzschild metric.
    static Vector3 ComputeAcceleration(const Vector3 &pos, const Vector3 &vel, const SimulationParams &params)
    {
        float r2 = Vector3LengthSqr(pos); // Distance squared from the center (r^2)
        float r = sqrtf(r2);              // Absolute distance (r)

        // Safety check: Prevent division by zero if the test mass falls perfectly into the singularity
        if (r < 0.5f)
            return {0, 0, 0};

        // Calculate specific angular momentum (L = r x v)
        Vector3 L = Vector3CrossProduct(pos, vel);
        float L2 = Vector3LengthSqr(L);
        float c2 = params.speedOfLight * params.speedOfLight;

        // 1. Classical Newtonian Gravity: a = GM / r^2
        float newtonianTerm = params.primaryMass / (r2 * r);

        // 2. General Relativity Correction Term: 1 + (3 * L^2) / (c^2 * r^2)
        // This extra term causes the famous "precession of the perihelion" (like Mercury's orbit)
        // and allows for orbits to decay into the black hole if they get too close.
        float grCorrection = 1.0f + (3.0f * L2) / (c2 * r2);

        // Combine them and point the acceleration vector back towards the origin (0,0,0)
        float magnitude = newtonianTerm * grCorrection;
        return Vector3Scale(pos, -magnitude);
    }

    // 4th-Order Runge-Kutta (RK4) Integrator
    // This is a highly accurate numerical method for predicting the next position of the object.
    // It takes 4 'samples' of the acceleration across the timestep to calculate a smooth curve.
    static void RK4Step(State &state, const SimulationParams &params)
    {
        float dt = params.timeStep;

        // k1: Initial slope at the start of the timestep
        Vector3 k1_v = ComputeAcceleration(state.pos, state.vel, params);
        Vector3 k1_p = state.vel;

        // k2: Slope at the midpoint (using k1)
        Vector3 k2_pos = Vector3Add(state.pos, Vector3Scale(k1_p, dt * 0.5f));
        Vector3 k2_vel = Vector3Add(state.vel, Vector3Scale(k1_v, dt * 0.5f));
        Vector3 k2_v = ComputeAcceleration(k2_pos, k2_vel, params);
        Vector3 k2_p = k2_vel;

        // k3: Another slope at the midpoint (using k2)
        Vector3 k3_pos = Vector3Add(state.pos, Vector3Scale(k2_p, dt * 0.5f));
        Vector3 k3_vel = Vector3Add(state.vel, Vector3Scale(k2_v, dt * 0.5f));
        Vector3 k3_v = ComputeAcceleration(k3_pos, k3_vel, params);
        Vector3 k3_p = k3_vel;

        // k4: Slope at the very end of the timestep (using k3)
        Vector3 k4_pos = Vector3Add(state.pos, Vector3Scale(k3_p, dt));
        Vector3 k4_vel = Vector3Add(state.vel, Vector3Scale(k3_v, dt));
        Vector3 k4_v = ComputeAcceleration(k4_pos, k4_vel, params);
        Vector3 k4_p = k4_vel;

        // Combine the 4 slopes using the RK4 weighted average formula
        Vector3 dv = Vector3Scale(Vector3Add(Vector3Add(k1_v, Vector3Scale(k2_v, 2.0f)), Vector3Add(Vector3Scale(k3_v, 2.0f), k4_v)), dt / 6.0f);
        Vector3 dp = Vector3Scale(Vector3Add(Vector3Add(k1_p, Vector3Scale(k2_p, 2.0f)), Vector3Add(Vector3Scale(k3_p, 2.0f), k4_p)), dt / 6.0f);

        // Apply the final changes to our test mass
        state.vel = Vector3Add(state.vel, dv);
        state.pos = Vector3Add(state.pos, dp);
    }
};

// ============================================================================
// --- VISUALIZATION ENGINE ---
// ============================================================================

// Draws the 3D wireframe representing the curvature of spacetime (Embedding Diagram)
void DrawEmbeddingDiagram(const SimulationParams &params)
{
    int gridSize = 40;    // Number of lines in the grid
    float spacing = 2.0f; // Space between each line
    Color gridColor = Fade(GREEN, 0.4f);

    // Lambda function to calculate how far 'down' the grid should bend at any x,y coordinate.
    // **UPDATE:** This now actively scales with the 'primaryMass' slider.
    auto getZ = [&](float x, float y) -> float
    {
        float r = sqrtf(x * x + y * y); // Distance from the central mass

        // Normalize the mass factor (baseline is 1000.0f)
        // If mass increases, massFactor goes > 1.0, deepening the well.
        float massFactor = params.primaryMass / 1000.0f;

        // Calculate the depth. We multiply warpDepth by our massFactor.
        // We add 2.0f to the denominator to prevent a sharp spike at r=0.
        return -(params.warpDepth * massFactor) / (r * params.gridElasticity + 2.0f);
    };

    // Draw the grid line by line
    for (int i = -gridSize; i < gridSize; i++)
    {
        for (int j = -gridSize; j < gridSize; j++)
        {
            // Point A (Current point)
            float x1 = i * spacing;
            float y1 = j * spacing;
            float z1 = getZ(x1, y1);

            // Point B (One step to the right)
            float x2 = (i + 1) * spacing;
            float y2 = j * spacing;
            float z2 = getZ(x2, y2);

            // Point C (One step forward)
            float x3 = i * spacing;
            float y3 = (j + 1) * spacing;
            float z3 = getZ(x3, y3);

            // Draw connecting lines
            DrawLine3D({x1, z1, y1}, {x2, z2, y2}, gridColor);
            DrawLine3D({x1, z1, y1}, {x3, z3, y3}, gridColor);
        }
    }
}

// ============================================================================
// --- MAIN APPLICATION LOOP ---
// ============================================================================

int main()
{
    // Window Setup
    const int screenWidth = 1600;
    const int screenHeight = 900;
    InitWindow(screenWidth, screenHeight, "General Relativity Simulation - C++20 RK4");
    SetTargetFPS(60);

    // Initialize the ImGui binding for Raylib
    rlImGuiSetup(true);

    // 3D Camera Setup
    Camera3D camera = {0};
    camera.position = {0.0f, 40.0f, 60.0f}; // Start high and pulled back
    camera.target = {0.0f, 0.0f, 0.0f};     // Looking at the center
    camera.up = {0.0f, 1.0f, 0.0f};         // Which way is 'up'
    camera.fovy = 45.0f;                    // Field of view
    camera.projection = CAMERA_PERSPECTIVE;

    // Initialize our physics objects
    SimulationParams params;
    State testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 6.0f}}; // Initial position and velocity
    std::vector<Vector3> orbitTrail;                            // Stores the past positions to draw the red line

    // Variables to hold the state of the interactive quiz
    int q1_ans = -1;
    int q2_ans = -1;
    char q3_ans[64] = "";
    std::string quizFeedback = "";

    // The Main Rendering Loop
    while (!WindowShouldClose())
    {
        // Allow the user to drag the mouse to look around
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // 1. UPDATE PHYSICS
        GRPhysicsEngine::RK4Step(testMass, params);

        // 2. UPDATE ORBIT TRAIL
        orbitTrail.push_back(testMass.pos);
        if (orbitTrail.size() > 200) // Keep the trail from getting infinitely long
            orbitTrail.erase(orbitTrail.begin());

        // 3. DRAW 3D SCENE
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        // Render the central object and spacetime
        if (!params.blackHoleMode)
        {
            // Standard Mode: Draw the grid and a Yellow Sun
            DrawEmbeddingDiagram(params);
            DrawSphere({0, 0, 0}, 3.0f, YELLOW);
        }
        else
        {
            // Black Hole Mode
            DrawSphere({0, 0, 0}, 4.0f, BLACK);                                      // The Event Horizon
            DrawCylinder({0, -0.1f, 0}, 15.0f, 15.0f, 0.2f, 32, Fade(ORANGE, 0.6f)); // Glowing Accretion Disk
            DrawSphereWires({0, 0, 0}, 6.0f, 16, 16, Fade(WHITE, 0.2f));             // Photon Sphere visualization
        }

        // Draw the test mass (the little red ball)
        DrawSphere(testMass.pos, 0.8f, RED);

        // Draw the fading red tail behind the test mass
        for (size_t i = 1; i < orbitTrail.size(); i++)
        {
            DrawLine3D(orbitTrail[i - 1], orbitTrail[i], Fade(RED, (float)i / orbitTrail.size()));
        }

        EndMode3D();

        // 4. DRAW 2D UI (ImGui)
        rlImGuiBegin();

        // --- Panel 1: Simulation Controls (Top Left) ---
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Simulation Controls");
        // These sliders automatically modify the values inside our 'params' struct
        ImGui::SliderFloat("Primary Mass (M)", &params.primaryMass, 100.0f, 5000.0f);
        ImGui::SliderFloat("Grid Warp Depth", &params.warpDepth, 10.0f, 150.0f);
        ImGui::SliderFloat("Grid Elasticity", &params.gridElasticity, 0.1f, 3.0f);
        ImGui::Checkbox("Black Hole Mode", &params.blackHoleMode);

        if (ImGui::Button("Reset Orbit"))
        {
            // Puts the ball back at the starting line
            testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 6.0f}};
            orbitTrail.clear();
        }
        ImGui::End();

        // --- Panel 2: Educational Info (Bottom Left) ---
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
        ImGui::Begin("Spacetime & Relativity");
        ImGui::TextWrapped("According to General Relativity, gravity is not a force, but a curvature of spacetime caused by mass and energy.");
        ImGui::Spacing();
        ImGui::Text("Einstein Field Equations:");
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "G_uv + ^ g_uv = (8piG / c^4) T_uv");
        ImGui::Spacing();
        ImGui::TextWrapped("The red sphere's path (geodesic) is calculated using a 4th-order Runge-Kutta integrator applying the Schwarzschild metric's effective potential.");
        ImGui::TextWrapped("Try increasing the mass. Notice how the spacetime grid deepens and the red sphere is pulled into a tighter orbit!");
        ImGui::End();

        // --- Panel 3: Interactive Quiz (Top Right) ---
        ImGui::SetNextWindowPos(ImVec2(screenWidth - 350, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Relativity Quiz");
        ImGui::Text("1. What parameter determines the radius of the event horizon?");
        ImGui::RadioButton("Schwarzschild Radius", &q1_ans, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Bohr Radius", &q1_ans, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Lorentz Factor", &q1_ans, 2);

        ImGui::Spacing();
        ImGui::Text("2. The visual distortion of light around a black hole is called:");
        ImGui::RadioButton("Gravitational Lensing", &q2_ans, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Redshifting", &q2_ans, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Time Dilation", &q2_ans, 2);

        ImGui::Spacing();
        ImGui::Text("3. Gravity is the curvature of ________ (Fill in the blank)");
        ImGui::InputText("##q3", q3_ans, IM_ARRAYSIZE(q3_ans));

        if (ImGui::Button("Submit Answers"))
        {
            int score = 0;
            if (q1_ans == 0)
                score++;
            if (q2_ans == 0)
                score++;

            std::string ans3 = q3_ans;
            for (auto &c : ans3)
                c = tolower(c); // Convert answer to lowercase to check it safely
            if (ans3.find("spacetime") != std::string::npos || ans3.find("space-time") != std::string::npos)
                score++;

            quizFeedback = "You scored " + std::to_string(score) + " out of 3!";
        }

        if (!quizFeedback.empty())
        {
            // Display the score in green
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "%s", quizFeedback.c_str());
        }
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    // Cleanup and Exit
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}