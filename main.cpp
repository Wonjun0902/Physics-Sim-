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

// ============================================================================
// --- NUMERICAL INTEGRATOR (PHYSICS ENGINE) ---
// ============================================================================

class GRPhysicsEngine
{
public:
    static Vector3 ComputeAcceleration(const Vector3 &pos, const Vector3 &vel, const SimulationParams &params)
    {
        float r2 = Vector3LengthSqr(pos);
        float r = sqrtf(r2);

        if (r < 0.5f)
            return {0, 0, 0};

        Vector3 L = Vector3CrossProduct(pos, vel);
        float L2 = Vector3LengthSqr(L);
        float c2 = params.speedOfLight * params.speedOfLight;

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

        Vector3 dv = Vector3Scale(Vector3Add(Vector3Add(k1_v, Vector3Scale(k2_v, 2.0f)), Vector3Add(Vector3Scale(k3_v, 2.0f), k4_v)), dt / 6.0f);
        Vector3 dp = Vector3Scale(Vector3Add(Vector3Add(k1_p, Vector3Scale(k2_p, 2.0f)), Vector3Add(Vector3Scale(k3_p, 2.0f), k4_p)), dt / 6.0f);

        state.vel = Vector3Add(state.vel, dv);
        state.pos = Vector3Add(state.pos, dp);
    }
};

// ============================================================================
// --- VISUALIZATION ENGINE ---
// ============================================================================

// Helper function to calculate the visual depth (Y-axis) of the gravity well at any physical X/Z coordinate
float GetVisualDepth(float physicalX, float physicalZ, const SimulationParams &params)
{
    float r = sqrtf(physicalX * physicalX + physicalZ * physicalZ);
    float massFactor = params.primaryMass / 1000.0f;
    return -(params.warpDepth * massFactor) / (r * params.gridElasticity + 2.0f);
}

void DrawEmbeddingDiagram(const SimulationParams &params)
{
    int gridSize = 40;
    float spacing = 2.0f;
    Color gridColor = Fade(GREEN, 0.4f);

    for (int i = -gridSize; i < gridSize; i++)
    {
        for (int j = -gridSize; j < gridSize; j++)
        {
            float x1 = i * spacing;
            float y1 = j * spacing;
            float z1 = GetVisualDepth(x1, y1, params);

            float x2 = (i + 1) * spacing;
            float y2 = j * spacing;
            float z2 = GetVisualDepth(x2, y2, params);

            float x3 = i * spacing;
            float y3 = (j + 1) * spacing;
            float z3 = GetVisualDepth(x3, y3, params);

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
    const int screenWidth = 1600;
    const int screenHeight = 900;
    InitWindow(screenWidth, screenHeight, "General Relativity Simulation - C++20 RK4");
    SetTargetFPS(60);

    rlImGuiSetup(true);

    // --> FONT SCALE FIX: Scales up all UI text globally
    ImGui::GetIO().FontGlobalScale = 1.5f;

    Camera3D camera = {0};
    camera.position = {0.0f, 40.0f, 60.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SimulationParams params;
    State testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 6.0f}};
    std::vector<Vector3> orbitTrail;

    int q1_ans = -1;
    int q2_ans = -1;
    char q3_ans[64] = "";
    std::string quizFeedback = "";

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // 1. UPDATE PHYSICS
        GRPhysicsEngine::RK4Step(testMass, params);

        // 2. UPDATE ORBIT TRAIL
        orbitTrail.push_back(testMass.pos);
        if (orbitTrail.size() > 200)
            orbitTrail.erase(orbitTrail.begin());

        // 3. DRAW 3D SCENE
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        // Get the depth of the exact center of the grid to place the massive objects
        float centerDepth = GetVisualDepth(0.0f, 0.0f, params);

        if (!params.blackHoleMode)
        {
            DrawEmbeddingDiagram(params);
            // Push the sun down into the well
            DrawSphere({0, centerDepth, 0}, 3.0f, YELLOW);
        }
        else
        {
            // Push the black hole and its disk down into the well
            DrawSphere({0, centerDepth, 0}, 4.0f, BLACK);
            DrawCylinder({0, centerDepth - 0.1f, 0}, 15.0f, 15.0f, 0.2f, 32, Fade(ORANGE, 0.6f));
            DrawSphereWires({0, centerDepth, 0}, 6.0f, 16, 16, Fade(WHITE, 0.2f));
        }

        // Project the test mass's Y coordinate down onto the grid
        float massDepth = GetVisualDepth(testMass.pos.x, testMass.pos.z, params);
        DrawSphere({testMass.pos.x, massDepth, testMass.pos.z}, 0.8f, RED);

        // Map every segment of the trail down onto the grid
        for (size_t i = 1; i < orbitTrail.size(); i++)
        {
            Vector3 p1 = {orbitTrail[i - 1].x, GetVisualDepth(orbitTrail[i - 1].x, orbitTrail[i - 1].z, params), orbitTrail[i - 1].z};
            Vector3 p2 = {orbitTrail[i].x, GetVisualDepth(orbitTrail[i].x, orbitTrail[i].z, params), orbitTrail[i].z};
            DrawLine3D(p1, p2, Fade(RED, (float)i / orbitTrail.size()));
        }

        EndMode3D();

        // 4. DRAW 2D UI (ImGui)
        rlImGuiBegin();

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

        // --> WINDOW SIZE FIX: Forces the window to stay small and adds a scrollbar
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(500, 250), ImGuiCond_Always);

        ImGui::Begin("Spacetime & Relativity");
        ImGui::TextWrapped("According to General Relativity, gravity is not a force, but a curvature of spacetime caused by mass and energy.");
        ImGui::Spacing();
        ImGui::Text("Einstein Field Equations:");
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "G_uv + ^ g_uv = (8piG / c^4) T_uv");
        ImGui::Spacing();
        ImGui::TextWrapped("The red sphere's path (geodesic) is calculated using a 4th-order Runge-Kutta integrator applying the Schwarzschild metric's effective potential.");
        ImGui::TextWrapped("Try increasing the mass. Notice how the spacetime grid deepens and the red sphere is pulled into a tighter orbit!");

        // You can put all your historical Einstein text here! ImGui will automatically let you scroll down to read it.
        ImGui::TextWrapped("General relativity was published by Albert Einstein in 1915-1916. It is the modern geometric theory of gravitation that describes gravity not as a force, but as a curvature of four-dimensional spacetime caused by mass and energy.");
        ImGui::TextWrapped("It generalizes special relativity, explaining that massive objects warp spacetime, affecting the path of objects and the flow of time.");
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(screenWidth - 450, 10), ImGuiCond_FirstUseEver);
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