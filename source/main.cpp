/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#ifdef TARGET_OS_MAC
#include <OpenGL/GL.h>
#else
#include <GL/gl.h>
#endif

#include <SDL2/SDL.h>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "maths.h"
#include "solver.h"
#include "scenes.h"

typedef int32_t i32;
typedef uint32_t u32;
typedef int32_t b32;

#define WinWidth 1280
#define WinHeight 720

b32 Running = 1;
b32 FullScreen = 0;
SDL_Window *Window;
SDL_GLContext Context;
u32 WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;

Solver* solver = new Solver();
Joint* drag = 0;
float camZoom = 25.0f;
float2 camPos = { 0, 5 };
int currScene = 4;
float2 boxSize = { 1, 1 };
float2 boxVelocity = { 0, 0 };
float boxFriction = 0.5f;
float boxDensity = 1.0f;
bool paused = false;

void ui()
{
    // Draw the ImGui UI
    ImGui::Begin("Controls");
    ImGui::Text("Move Cam: W,A,S,D / Middle Mouse");
    ImGui::Text("Zoom Cam: Q,E / Mouse Wheel");
    ImGui::Text("Make Box: Right Mouse");
    ImGui::Text("Drag Box: Left Mouse");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    int scene = currScene;
    if (ImGui::BeginCombo("Scene", sceneNames[scene]))
    {
        for (int i = 0; i < sceneCount; i++)
        {
            bool selected = scene == i;
            if (ImGui::Selectable(sceneNames[i], selected) && i != currScene)
            {
                currScene = i;
                scenes[currScene](solver);
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (ImGui::Button(" Reset "))
        scenes[currScene](solver);
    ImGui::SameLine();
    if (ImGui::Button("Default"))
        solver->defaultParams();

    ImGui::Checkbox("Pause", &paused);
    if (paused)
    {
        ImGui::SameLine();
        if (ImGui::Button("Step"))
            solver->step();
    }

    ImGui::Spacing();
    ImGui::SliderFloat("Box Friction", &boxFriction, 0.0f, 2.0f);
    ImGui::SliderFloat2("Box Size", &boxSize.x, 0.1f, 10.0f);
    ImGui::SliderFloat2("Box Velocity", &boxVelocity.x, -20.0f, 20.0f);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::SliderFloat("Gravity", &solver->gravity, -20.0f, 20.0f);
    ImGui::SliderFloat("Dt", &solver->dt, 0.001f, 0.1f);
    ImGui::SliderInt("Iterations", &solver->iterations, 1, 50);

    if (!solver->postStabilize)
        ImGui::SliderFloat("Alpha", &solver->alpha, 0.0f, 1.0f);
    ImGui::SliderFloat("Beta", &solver->beta, 0.0f, 1000000.0f, "%.f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Gamma", &solver->gamma, 0.0f, 1.0f);

    ImGui::Checkbox("Post Stabilize", &solver->postStabilize);

    ImGui::End();
}

void input()
{
    auto& io = ImGui::GetIO();

    // Convert mouse position to world coordinates
    float2 mousePos = camPos + (float2{ io.MousePos.x, io.DisplaySize.y - io.MousePos.y } -
        float2{ io.DisplaySize.x, io.DisplaySize.y } *0.5f) / camZoom;

    // Camera keyboard controls
    if (ImGui::IsKeyDown(ImGuiKey_D))
        camPos.x += 10 / camZoom;
    if (ImGui::IsKeyDown(ImGuiKey_A))
        camPos.x -= 10 / camZoom;
    if (ImGui::IsKeyDown(ImGuiKey_W))
        camPos.y += 10 / camZoom;
    if (ImGui::IsKeyDown(ImGuiKey_S))
        camPos.y -= 10 / camZoom;
    if (ImGui::IsKeyDown(ImGuiKey_E))
        camZoom *= 1.025f;
    if (ImGui::IsKeyDown(ImGuiKey_Q))
        camZoom /= 1.025f;

    // Camera mouse controls
    if (ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        camPos -= float2{ io.MouseDelta.x, -io.MouseDelta.y } / camZoom;
    camZoom *= powf(1.1f, io.MouseWheel);

    // Draw box
    if (io.MouseDown[ImGuiMouseButton_Left])
    {
        if (!drag)
        {
            float2 local;
            Rigid* b;
            if ((b = solver->pick(mousePos, local)))
                drag = new Joint(solver, 0, b, mousePos, local, float3{ 1000.0f, 1000.0f, 0.0f });
        }
        else
            drag->rA = mousePos;
    }
    else if (drag)
    {
        delete drag;
        drag = 0;
    }

    // Create box
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        new Rigid(solver, boxSize, boxDensity, boxFriction, float3{ mousePos.x, mousePos.y, 0.0f },
            float3{ boxVelocity.x, boxVelocity.y, 0.0f });
    }
}

void mainLoop()
{
    // Event loop
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL2_ProcessEvent(&event);

        if (event.type == SDL_KEYDOWN)
        {
            if (event.key.keysym.sym == SDLK_RETURN && (event.key.keysym.mod & KMOD_ALT))
            {
                FullScreen = !FullScreen;
                Uint32 fullscreenFlag = FullScreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0;
                SDL_SetWindowFullscreen(Window, fullscreenFlag);
            }
            
            if (event.key.keysym.sym == SDLK_ESCAPE)
            {
                #ifndef __EMSCRIPTEN__
                Running = 0;
                #endif
            }
        }
        else if (event.type == SDL_QUIT)
        { 
            #ifndef __EMSCRIPTEN__
            Running = 0;
            #endif
        }
    }

    // Setup GL
    int w, h;
    SDL_GetWindowSize(Window, &w, &h);

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);
    glPointSize(3.0f);
    glViewport(0, 0, w, h);
    glClearColor(1, 1, 1, 1); 
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(camPos.x - w / 2 / camZoom, camPos.x + w / 2 / camZoom, camPos.y - h / 2 / camZoom, camPos.y + h / 2 / camZoom, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ImGUI setup
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    input();
    ui();

    // Step solver and draw it
    if (!paused)
        solver->step();
    solver->draw();

    // ImGUI rendering
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(Window);
}

int main(int argc, char* argv[])
{
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);    // Enable multisampling
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    #ifdef EMSCRIPTEN
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);  // OpenGL ES 3.0 (WebGL2)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);  // No forward-compatible flag
    #endif

    // Create the SDL window
    Window = SDL_CreateWindow("AVBD 2D", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WinWidth, WinHeight, WindowFlags);
    if (!Window)
    {
        printf("Failed to create window: %s\n", SDL_GetError());
        return -1;
    }

    // Create the OpenGL context
    Context = SDL_GL_CreateContext(Window);
    if (!Context)
    {
        printf("Failed to create OpenGL context: %s\n", SDL_GetError());
        SDL_DestroyWindow(Window);
        SDL_Quit();
        return -1;
    }
    
    SDL_GL_MakeCurrent(Window, Context);
    SDL_GL_SetSwapInterval(1); // Enable vsync
    
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForOpenGL(Window, Context);
    #ifdef __EMSCRIPTEN__
    ImGui_ImplOpenGL3_Init("#version 300 es");  // WebAssembly
    #else
    ImGui_ImplOpenGL3_Init("#version 150");     // Desktop OpenGL
    #endif

    // Load scene
    scenes[currScene](solver);

    #ifdef __EMSCRIPTEN__
    // Use Emscripten's main loop for the web
    emscripten_set_main_loop(mainLoop, 0, 1);
    #else
    // For native builds, use a while loop
    while (Running)
    {
        mainLoop();
    }
    #endif

    // Cleanup
    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}
