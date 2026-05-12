#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "rlImGui.h"
#include <vector>
#include <string>
#include <cmath>

// --- Physics Data Structures ---
struct State
{
    Vector3 pos;
    Vector3 vel;
};

struct SimulationParams
{
    float primaryMass = 1000.0f;
    float warpDepth = 50.0f;
    float speedOfLight = 100.0f;
    float gridElasticity = 1.0f;
    bool blackHoleMode = false;
    float timeStep = 0.016f;
};

// --- RK4 Integrator ---
class GRPhysicsEngine
{
public:
    static Vector3 ComputeAcceleration(const Vector3 &pos, const Vector3 &vel, const SimulationParams &params)
    {
        float r2 = Vector3LengthSqr(pos);
        float r = sqrtf(r2);

        if (r < 0.5f)
            return {0, 0, 0}; // Prevent singularity division

        Vector3 L = Vector3CrossProduct(pos, vel);
        float L2 = Vector3LengthSqr(L);
        float c2 = params.speedOfLight * params.speedOfLight;

        // Schwarzschild modified acceleration
        float newtonianTerm = params.primaryMass / (r2 * r);
        float grCorrection = 1.0f + (3.0f * L2) / (c2 * r2);

        float magnitude = newtonianTerm * grCorrection;
        return Vector3Scale(pos, -magnitude);
    }

    static void RK4Step(State &state, const SimulationParams &params)
    {
        float dt = params.timeStep;

        Vector3 k1_v = ComputeAcceleration(state.pos, state.vel, params);
        Vector3 k1_p = state.vel;

        Vector3 k2_pos = Vector3Add(state.pos, Vector3Scale(k1_p, dt * 0.5f));
        Vector3 k2_vel = Vector3Add(state.vel, Vector3Scale(k1_v, dt * 0.5f));
        Vector3 k2_v = ComputeAcceleration(k2_pos, k2_vel, params);
        Vector3 k2_p = k2_vel;

        Vector3 k3_pos = Vector3Add(state.pos, Vector3Scale(k2_p, dt * 0.5f));
        Vector3 k3_vel = Vector3Add(state.vel, Vector3Scale(k2_v, dt * 0.5f));
        Vector3 k3_v = ComputeAcceleration(k3_pos, k3_vel, params);
        Vector3 k3_p = k3_vel;

        Vector3 k4_pos = Vector3Add(state.pos, Vector3Scale(k3_p, dt));
        Vector3 k4_vel = Vector3Add(state.vel, Vector3Scale(k3_v, dt));
        Vector3 k4_v = ComputeAcceleration(k4_pos, k4_vel, params);
        Vector3 k4_p = k4_vel;

        // Combine
        Vector3 dv = Vector3Scale(Vector3Add(Vector3Add(k1_v, Vector3Scale(k2_v, 2.0f)), Vector3Add(Vector3Scale(k3_v, 2.0f), k4_v)), dt / 6.0f);
        Vector3 dp = Vector3Scale(Vector3Add(Vector3Add(k1_p, Vector3Scale(k2_p, 2.0f)), Vector3Add(Vector3Scale(k3_p, 2.0f), k4_p)), dt / 6.0f);

        state.vel = Vector3Add(state.vel, dv);
        state.pos = Vector3Add(state.pos, dp);
    }
};

// --- Visualization Engine ---
void DrawEmbeddingDiagram(const SimulationParams &params)
{
    int gridSize = 40;
    float spacing = 2.0f;
    Color gridColor = Fade(GREEN, 0.4f);

    auto getZ = [&](float x, float y) -> float
    {
        float r = sqrtf(x * x + y * y);
        // Softened gravity well for visual diagram
        return -params.warpDepth / (r * params.gridElasticity + 2.0f);
    };

    for (int i = -gridSize; i < gridSize; i++)
    {
        for (int j = -gridSize; j < gridSize; j++)
        {
            float x1 = i * spacing;
            float y1 = j * spacing;
            float z1 = getZ(x1, y1);

            float x2 = (i + 1) * spacing;
            float y2 = j * spacing;
            float z2 = getZ(x2, y2);

            float x3 = i * spacing;
            float y3 = (j + 1) * spacing;
            float z3 = getZ(x3, y3);

            DrawLine3D({x1, z1, y1}, {x2, z2, y2}, gridColor);
            DrawLine3D({x1, z1, y1}, {x3, z3, y3}, gridColor);
        }
    }
}

// --- Main Application ---
int main()
{
    const int screenWidth = 1600;
    const int screenHeight = 900;
    InitWindow(screenWidth, screenHeight, "General Relativity Simulation - C++20 RK4");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Camera3D camera = {0};
    camera.position = {0.0f, 40.0f, 60.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SimulationParams params;
    State testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 6.0f}}; // Initial orbit
    std::vector<Vector3> orbitTrail;

    // Quiz State
    int q1_ans = -1;
    int q2_ans = -1;
    char q3_ans[64] = "";
    std::string quizFeedback = "";

    while (!WindowShouldClose())
    {
        // Update Camera
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // Physics Update
        GRPhysicsEngine::RK4Step(testMass, params);

        // Trail management
        orbitTrail.push_back(testMass.pos);
        if (orbitTrail.size() > 200)
            orbitTrail.erase(orbitTrail.begin());

        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        // Draw Spacetime Grid
        if (!params.blackHoleMode)
        {
            DrawEmbeddingDiagram(params);
            DrawSphere({0, 0, 0}, 3.0f, YELLOW); // Primary Mass (Sun)
        }
        else
        {
            // Black Hole Visuals (Placeholder for Volumetric/Raymarching Shader)
            DrawSphere({0, 0, 0}, 4.0f, BLACK);                                      // Event Horizon
            DrawCylinder({0, -0.1f, 0}, 15.0f, 15.0f, 0.2f, 32, Fade(ORANGE, 0.6f)); // Accretion Disk
            DrawSphereWires({0, 0, 0}, 6.0f, 16, 16, Fade(WHITE, 0.2f));             // Lensing photon sphere
        }

        // Draw Test Mass
        DrawSphere(testMass.pos, 0.8f, RED);

        // Draw Orbit Trail
        for (size_t i = 1; i < orbitTrail.size(); i++)
        {
            DrawLine3D(orbitTrail[i - 1], orbitTrail[i], Fade(RED, (float)i / orbitTrail.size()));
        }

        EndMode3D();

        // UI Overlay
        rlImGuiBegin();

        // Position at Top Left
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Simulation Controls");
        ImGui::SliderFloat("Primary Mass (M)", &params.primaryMass, 100.0f, 5000.0f);
        ImGui::SliderFloat("Grid Warp Depth", &params.warpDepth, 10.0f, 150.0f);
        ImGui::SliderFloat("Grid Elasticity", &params.gridElasticity, 0.1f, 3.0f);
        ImGui::Checkbox("Black Hole Mode", &params.blackHoleMode);

        if (ImGui::Button("Reset Orbit"))
        {
            testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 6.0f}};
            orbitTrail.clear();
        }
        ImGui::End();

        // Position at Bottom Left
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
        ImGui::Begin("Spacetime & Relativity");
        ImGui::TextWrapped("According to General Relativity, gravity is not a force, but a curvature of spacetime caused by mass and energy.");
        ImGui::Spacing();
        ImGui::Text("Einstein Field Equations:");
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "G_uv + ^ g_uv = (8piG / c^4) T_uv");
        ImGui::Spacing();
        ImGui::TextWrapped("The red sphere's path (geodesic) is calculated using a 4th-order Runge-Kutta integrator applying the Schwarzschild metric's effective potential.");
        ImGui::End();

        // Position at Top Right
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
            // Basic string normalization for checking
            for (auto &c : ans3)
                c = tolower(c);
            if (ans3.find("spacetime") != std::string::npos || ans3.find("space-time") != std::string::npos)
                score++;

            quizFeedback = "You scored " + std::to_string(score) + " out of 3!";
        }
        if (!quizFeedback.empty())
        {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "%s", quizFeedback.c_str());
        }
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}