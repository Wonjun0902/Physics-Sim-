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

    // Time variables
    float timeStep = 0.016f;
    float elapsedTime = 0.0f;

    // Circular Motion variables
    float primaryOrbitRadius = 0.0f;
    float primaryOrbitSpeed = 0.5f;
};

// ============================================================================
// --- NUMERICAL INTEGRATOR (PHYSICS ENGINE) ---
// ============================================================================

class GRPhysicsEngine
{
public:
    static Vector3 ComputeAcceleration(const Vector3 &testPos, const Vector3 &testVel, const Vector3 &primaryPos, const SimulationParams &params)
    {
        Vector3 r_vec = Vector3Subtract(testPos, primaryPos);
        float r2 = Vector3LengthSqr(r_vec);
        float r = sqrtf(r2);

        // Extreme safety fallback to prevent dividing by zero
        if (r < 0.1f)
            return {0, 0, 0};

        Vector3 L = Vector3CrossProduct(r_vec, testVel);
        float L2 = Vector3LengthSqr(L);
        float c2 = params.speedOfLight * params.speedOfLight;

        float newtonianTerm = params.primaryMass / (r2 * r);
        float grCorrection = 1.0f + (3.0f * L2) / (c2 * r2);

        float magnitude = newtonianTerm * grCorrection;
        return Vector3Scale(r_vec, -magnitude);
    }

    // NEW: Handles Substepping and the Event Horizon collision
    static void UpdatePhysics(State &test, State &primary, SimulationParams &params)
    {
        // 1. Physics Substepping: Calculate math 10 times per frame for extreme stability
        int substeps = 10;
        float dt = params.timeStep / substeps;

        for (int i = 0; i < substeps; i++)
        {
            params.elapsedTime += dt;

            // 2. Move Primary Mass in a Circle using Trigonometry
            primary.pos.x = params.primaryOrbitRadius * cosf(params.elapsedTime * params.primaryOrbitSpeed);
            primary.pos.z = params.primaryOrbitRadius * sinf(params.elapsedTime * params.primaryOrbitSpeed);
            primary.vel.x = -params.primaryOrbitRadius * params.primaryOrbitSpeed * sinf(params.elapsedTime * params.primaryOrbitSpeed);
            primary.vel.z = params.primaryOrbitRadius * params.primaryOrbitSpeed * cosf(params.elapsedTime * params.primaryOrbitSpeed);

            // 3. Collision / Absorption Check
            Vector3 r_vec = Vector3Subtract(test.pos, primary.pos);
            float r = Vector3Length(r_vec);

            // Set the "capture" radius slightly smaller than the visual models
            float captureRadius = params.blackHoleMode ? 3.8f : 2.8f;

            if (r < captureRadius)
            {
                // The object has fallen in! Lock it to the primary mass
                test.pos = primary.pos;
                test.vel = primary.vel;
                continue; // Skip the RK4 math, it's trapped
            }

            // 4. Standard RK4 Integration using our highly stable 'dt'
            Vector3 k1_v = ComputeAcceleration(test.pos, test.vel, primary.pos, params);
            Vector3 k1_p = test.vel;

            Vector3 k2_pos = Vector3Add(test.pos, Vector3Scale(k1_p, dt * 0.5f));
            Vector3 k2_vel = Vector3Add(test.vel, Vector3Scale(k1_v, dt * 0.5f));
            Vector3 k2_v = ComputeAcceleration(k2_pos, k2_vel, primary.pos, params);
            Vector3 k2_p = k2_vel;

            Vector3 k3_pos = Vector3Add(test.pos, Vector3Scale(k2_p, dt * 0.5f));
            Vector3 k3_vel = Vector3Add(test.vel, Vector3Scale(k2_v, dt * 0.5f));
            Vector3 k3_v = ComputeAcceleration(k3_pos, k3_vel, primary.pos, params);
            Vector3 k3_p = k3_vel;

            Vector3 k4_pos = Vector3Add(test.pos, Vector3Scale(k3_p, dt));
            Vector3 k4_vel = Vector3Add(test.vel, Vector3Scale(k3_v, dt));
            Vector3 k4_v = ComputeAcceleration(k4_pos, k4_vel, primary.pos, params);
            Vector3 k4_p = k4_vel;

            Vector3 dv = Vector3Scale(Vector3Add(Vector3Add(k1_v, Vector3Scale(k2_v, 2.0f)), Vector3Add(Vector3Scale(k3_v, 2.0f), k4_v)), dt / 6.0f);
            Vector3 dp = Vector3Scale(Vector3Add(Vector3Add(k1_p, Vector3Scale(k2_p, 2.0f)), Vector3Add(Vector3Scale(k3_p, 2.0f), k4_p)), dt / 6.0f);

            test.vel = Vector3Add(test.vel, dv);
            test.pos = Vector3Add(test.pos, dp);
        }
    }
};

// ============================================================================
// --- VISUALIZATION ENGINE ---
// ============================================================================

float GetVisualDepth(float physicalX, float physicalZ, const Vector3 &primaryPos, const SimulationParams &params)
{
    float dx = physicalX - primaryPos.x;
    float dz = physicalZ - primaryPos.z;
    float r = sqrtf(dx * dx + dz * dz);

    float massFactor = params.primaryMass / 1000.0f;
    return -(params.warpDepth * massFactor) / (r * params.gridElasticity + 2.0f);
}

void DrawEmbeddingDiagram(const Vector3 &primaryPos, const SimulationParams &params)
{
    int gridSize = 60;
    float spacing = 2.0f;
    Color gridColor = Fade(GREEN, 0.4f);

    for (int i = -gridSize; i < gridSize; i++)
    {
        for (int j = -gridSize; j < gridSize; j++)
        {
            float x1 = i * spacing;
            float y1 = j * spacing;
            float z1 = GetVisualDepth(x1, y1, primaryPos, params);
            float x2 = (i + 1) * spacing;
            float y2 = j * spacing;
            float z2 = GetVisualDepth(x2, y2, primaryPos, params);
            float x3 = i * spacing;
            float y3 = (j + 1) * spacing;
            float z3 = GetVisualDepth(x3, y3, primaryPos, params);

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
    ImGui::GetIO().FontGlobalScale = 1.5f;

    Camera3D camera = {0};
    camera.position = {0.0f, 60.0f, 80.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SimulationParams params;
    State primaryMass = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    State testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 10.0f}};

    std::vector<Vector3> orbitTrail;
    int q1_ans = -1;
    int q2_ans = -1;
    char q3_ans[64] = "";
    std::string quizFeedback = "";

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // 1. UPDATE PHYSICS WITH SUBSTEPPING
        GRPhysicsEngine::UpdatePhysics(testMass, primaryMass, params);

        // 2. UPDATE ORBIT TRAIL
        // Only draw the trail if it hasn't been captured by the black hole
        Vector3 distVec = Vector3Subtract(testMass.pos, primaryMass.pos);
        if (Vector3Length(distVec) > (params.blackHoleMode ? 3.8f : 2.8f))
        {
            orbitTrail.push_back(testMass.pos);
            if (orbitTrail.size() > 400)
                orbitTrail.erase(orbitTrail.begin());
        }

        // 3. DRAW 3D SCENE
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        float centerDepth = GetVisualDepth(primaryMass.pos.x, primaryMass.pos.z, primaryMass.pos, params);

        if (!params.blackHoleMode)
        {
            DrawEmbeddingDiagram(primaryMass.pos, params);
            DrawSphere({primaryMass.pos.x, centerDepth, primaryMass.pos.z}, 3.0f, YELLOW);
        }
        else
        {
            DrawEmbeddingDiagram(primaryMass.pos, params);
            DrawSphere({primaryMass.pos.x, centerDepth, primaryMass.pos.z}, 4.0f, BLACK);
            DrawCylinder({primaryMass.pos.x, centerDepth - 0.1f, primaryMass.pos.z}, 15.0f, 15.0f, 0.2f, 32, Fade(ORANGE, 0.6f));
            DrawSphereWires({primaryMass.pos.x, centerDepth, primaryMass.pos.z}, 6.0f, 16, 16, Fade(WHITE, 0.2f));
        }

        float massDepth = GetVisualDepth(testMass.pos.x, testMass.pos.z, primaryMass.pos, params);
        DrawSphere({testMass.pos.x, massDepth, testMass.pos.z}, 0.8f, RED);

        for (size_t i = 1; i < orbitTrail.size(); i++)
        {
            Vector3 p1 = {orbitTrail[i - 1].x, GetVisualDepth(orbitTrail[i - 1].x, orbitTrail[i - 1].z, primaryMass.pos, params), orbitTrail[i - 1].z};
            Vector3 p2 = {orbitTrail[i].x, GetVisualDepth(orbitTrail[i].x, orbitTrail[i].z, primaryMass.pos, params), orbitTrail[i].z};
            DrawLine3D(p1, p2, Fade(RED, (float)i / orbitTrail.size()));
        }

        EndMode3D();

        // 4. DRAW 2D UI (ImGui)
        rlImGuiBegin();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Simulation Controls");
        ImGui::SliderFloat("Primary Mass (M)", &params.primaryMass, 100.0f, 5000.0f);

        // NEW: Sliders for Circular Motion
        ImGui::SliderFloat("Primary Orbit Radius", &params.primaryOrbitRadius, 0.0f, 20.0f);
        ImGui::SliderFloat("Primary Orbit Speed", &params.primaryOrbitSpeed, 0.0f, 3.0f);

        ImGui::SliderFloat("Grid Warp Depth", &params.warpDepth, 10.0f, 150.0f);
        ImGui::Checkbox("Black Hole Mode", &params.blackHoleMode);

        if (ImGui::Button("Reset Orbit"))
        {
            primaryMass = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
            testMass = {{20.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 10.0f}};
            params.elapsedTime = 0.0f;
            orbitTrail.clear();
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(500, 250), ImGuiCond_Always);
        ImGui::Begin("Spacetime & Relativity");
        ImGui::TextWrapped("Gravity is the curvature of spacetime caused by mass and energy. When a massive object moves, the curvature of spacetime moves with it!");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "G_uv + ^ g_uv = (8piG / c^4) T_uv");
        ImGui::Spacing();
        ImGui::TextWrapped("Try adding 'Primary Orbit Radius'. You will see the gravity well travel in a circle, dragging the test mass with it! If the test mass gets too close, it will now realistically fall into the event horizon instead of shooting into deep space.");
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(screenWidth - 450, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Relativity Quiz");
        ImGui::Text("1. What parameter determines the radius of the event horizon?");
        ImGui::RadioButton("Schwarzschild Radius", &q1_ans, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Bohr Radius", &q1_ans, 1);
        ImGui::Spacing();
        ImGui::Text("2. The visual distortion of light around a black hole is called:");
        ImGui::RadioButton("Gravitational Lensing", &q2_ans, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Redshifting", &q2_ans, 1);
        ImGui::Spacing();
        ImGui::Text("3. Gravity is the curvature of ________");
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
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "%s", quizFeedback.c_str());
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}