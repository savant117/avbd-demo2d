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

#pragma once

#include "maths.h"
#include "solver.h"

static void sceneEmpty(Solver* solver)
{
    solver->clear();
}

static void sceneGround(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 }, { 0, 0, 0 });
}

static void sceneDynamicFriction(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 }, { 0, 0, 0 });
    for (int x = 0; x <= 10; x++)
        new Rigid(solver, { 1, 0.5f }, 1.0f, 5.0f - (x / 10.0f * 5.0f), { -30.0f + x * 2.0f, 0.75f, 0 }, { 10.0f, 0, 0 });
}

static void sceneStaticFriction(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 1 }, 0.0f, 1.0f, { 0, 0, 3.14159f / 6.0f });
    for (int y = 0; y <= 10; y++)
        new Rigid(solver, { 5, 0.5f }, 1.0f, 1.0f, { 0.0f, y * 1.0f + 1.0f, 3.14159f / 6.0f });
}

static void scenePyramid(Solver* solver)
{
    const int SIZE = 20;
    solver->clear();
    new Rigid(solver, { 100, 0.5f }, 0.0f, 0.5f, { 0.0f, -2.0f, 0.0f });
    for (int y = 0; y < SIZE; y++)
        for (int x = 0; x < SIZE - y; x++)
            new Rigid(solver, { 1, 0.5f }, 1.0f, 0.5f, { x * 1.1f + y * 0.5f - SIZE / 2.0f, y * 0.85f, 0.0f });
}

static void sceneRope(Solver* solver)
{
    solver->clear();
    Rigid* prev = 0;
    for (int i = 0; i < 20; i++)
    {
        Rigid* curr = new Rigid(solver, { 1, 0.5f }, i == 0 ? 0.0f : 1.0f, 0.5f, { (float)i, 10.0f, 0.0f });
        if (prev)
            new Joint(solver, prev, curr, { 0.5f, 0 }, { -0.5f, 0 }, { INFINITY, INFINITY, 0.0f });
        prev = curr;
    }
}

static void sceneHeavyRope(Solver* solver)
{
    const int N = 20;
    const float SIZE = 30;
    solver->clear();
    Rigid* prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid* curr = new Rigid(solver, i == N - 1 ? float2 { SIZE, SIZE } : float2 { 1, 0.5f }, i == 0 ? 0.0f : 1.0f, 0.5f, { (float)i + (i == N - 1 ? SIZE / 2 : 0), 10.0f, 0.0f});
        if (prev)
            new Joint(solver, prev, curr, { 0.5f, 0 }, i == N - 1 ? float2{ -SIZE / 2, 0 } : float2 { -0.5f, 0 }, { INFINITY, INFINITY, 0.0f });
        prev = curr;
    }
}

static void sceneHangingRope(Solver* solver)
{
    const int N = 50;
    const float SIZE = 10;
    solver->clear();
    Rigid* prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid* curr = new Rigid(solver, i == N - 1 ? float2{ SIZE, SIZE } : float2{ 0.5f, 1.0f }, i == 0 ? 0.0f : 1.0f, 0.5f, { 0.0f, 10.0f - ((float)i + (i == N - 1 ? SIZE / 2 : 0)), 0.0f });
        if (prev)
            new Joint(solver, prev, curr, { 0, -0.5f }, i == N - 1 ? float2{ 0, SIZE / 2} : float2{ 0, 0.5f }, { INFINITY, INFINITY, 0.0f });
        prev = curr;
    }
}

static void sceneSpring(Solver* solver)
{
    solver->clear();
    Rigid* anchor = new Rigid(solver, { 1, 1 }, 0.0f, 0.5f, { 0.0f, 0.0f, 0.0f });
    Rigid* block = new Rigid(solver, { 4, 4 }, 1.0f, 0.5f, { 0.0f, -8.0f, 0.0f });
    new Spring(solver, anchor, block, { 0, 0 }, { 0, 0 }, 100.0f, 4.0f);
}

static void sceneSpringsRatio(Solver* solver)
{
    const int N = 8;
    solver->clear();
    Rigid* prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid* curr = new Rigid(solver, { 1, 0.5f }, i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, { (float)i * 4, 10.0f, 0.0f });
        if (prev)
            new Spring(solver, prev, curr, { 0.5f, 0 }, { -0.5f, 0 }, i % 2 == 0 ? 1000.0f : 1000000.0f, 0.1f);
        prev = curr;
    }
}

static void sceneStack(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 });
    for (int i = 0; i < 20; i++)
        new Rigid(solver, { 1, 1 }, 1.0f, 0.5f, { 0, i * 2.0f + 1.0f, 0 });
}

static void sceneStackRatio(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 1 }, 0.0f, 0.5f, { 0, 0, 0 });
    for (int i = 0, y = 1, s = 1; i < 6; i++)
    {
        new Rigid(solver, { (float)s, (float)s }, 1.0f, 0.5f, { 0, (float)y, 0 });
        y += s * 3 / 2;
        s *= 2;
    }
}

static void sceneRod(Solver* solver)
{
    solver->clear();
    Rigid* prev = 0;
    for (int i = 0; i < 20; i++)
    {
        Rigid* curr = new Rigid(solver, { 1, 0.5f }, i == 0 ? 0.0f : 1.0f, 0.5f, { (float)i, 10.0f, 0.0f });
        if (prev)
            new Joint(solver, prev, curr, { 0.5f, 0 }, { -0.5f, 0 }, { INFINITY, INFINITY, INFINITY });
        prev = curr;
    }
}

static void sceneSoftBody(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 0.5f }, 0.0f, 0.5f, { 0.0f, 0.0f });

    const float Klin = 1000.0f;
    const float Kang = 100.0f;
    const int W = 15, H = 5;
    const int N = 2;
    for (int i = 0; i < N; i++)
    {
        Rigid* grid[W][H];
        for (int x = 0; x < W; x++)
            for (int y = 0; y < H; y++)
                grid[x][y] = new Rigid(solver, { 1, 1 }, 1.0f, 0.5f, { (float)x, (float)y + H * i * 2.0f + 5.0f, 0.0f });

        for (int x = 1; x < W; x++)
            for (int y = 0; y < H; y++)
                new Joint(solver, grid[x - 1][y], grid[x][y], { 0.5f, 0 }, { -0.5f, 0 }, { Klin, Klin, Kang });

        for (int x = 0; x < W; x++)
            for (int y = 1; y < H; y++)
                new Joint(solver, grid[x][y - 1], grid[x][y], { 0, 0.5f }, { 0, -0.5f }, { Klin, Klin, Kang });

        for (int x = 1; x < W; x++)
        {
            for (int y = 1; y < H; y++)
            {
                new IgnoreCollision(solver, grid[x - 1][y - 1], grid[x][y]);
                new IgnoreCollision(solver, grid[x][y - 1], grid[x - 1][y]);
            }
        }
    }
}

static void sceneJointGrid(Solver* solver)
{
    solver->clear();

    const int W = 25, H = 25;
    const int N = 2;

    Rigid* grid[W][H];
    for (int x = 0; x < W; x++)
        for (int y = 0; y < H; y++)
            grid[x][y] = new Rigid(solver, { 1, 1 }, y == H - 1 && (x == 0 || x == W - 1) ? 0.0f : 1.0f, 0.5f, { (float)x, (float)y, 0.0f });

    for (int x = 1; x < W; x++)
        for (int y = 0; y < H; y++)
            new Joint(solver, grid[x - 1][y], grid[x][y], { 0.5f, 0 }, { -0.5f, 0 });

    for (int x = 0; x < W; x++)
        for (int y = 1; y < H; y++)
            new Joint(solver, grid[x][y - 1], grid[x][y], { 0, 0.5f }, { 0, -0.5f });

    for (int x = 1; x < W; x++)
    {
        for (int y = 1; y < H; y++)
        {
            new IgnoreCollision(solver, grid[x - 1][y - 1], grid[x][y]);
            new IgnoreCollision(solver, grid[x][y - 1], grid[x - 1][y]);
        }
    }
}

static void sceneNet(Solver* solver)
{
    const int N = 40;

    solver->clear();
    new Rigid(solver, { 100, 0.5f }, 0.0f, 0.5f, { 0.0f, 0.0f });

    Rigid* prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid* curr = new Rigid(solver, { 1, 0.5f }, i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, { (float)i - N / 2.0f, 10.0f, 0.0f });
        if (prev)
            new Joint(solver, prev, curr, { 0.5f, 0 }, { -0.5f, 0 }, { INFINITY, INFINITY, 0.0f });
        prev = curr;
    }

    for (int x = 0; x < N / 4; x++)
        for (int y = 0; y < N / 8; y++)
            new Rigid(solver, { 1, 1 }, 1.0f, 0.5f, { (float)x - N / 8.0f, (float)y + 15.0f, 0.0f });
}

static void sceneMotor(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 100, 0.5f }, 0.0f, 0.5f, { 0.0f, -10.0f });

    Rigid* a0 = new Rigid(solver, { 5, 0.5f }, 1.0f, 0.5f, { 0.0f, 0.0f, 0.0f });
    new Joint(solver, 0, a0, { 0, 0 }, { 0, 0 }, { INFINITY, INFINITY, 0.0f });
    new Motor(solver, 0, a0, 20.0f, 50.0f);
}

static void sceneFracture(Solver* solver)
{
    const int N = 10;
    const int M = 15;

    solver->clear();
    new Rigid(solver, { 100, 0.5f }, 0.0f, 0.5f, { 0.0f, 0.0f });

    Rigid* prev = 0;
    for (int i = 0; i <= N; i++)
    {
        Rigid* curr = new Rigid(solver, { 1, 0.5f }, 1.0f, 0.5f, { (float)i - N / 2.0f, 6.0f, 0.0f });
        if (prev)
            new Joint(solver, prev, curr, { 0.5f, 0 }, { -0.5f, 0 }, { INFINITY, INFINITY, INFINITY }, 500.0f);
        prev = curr;
    }

    new Rigid(solver, { 1, 5 }, 1.0f, 0.5f, { -N / 2.0f, 2.5f, 0 });
    new Rigid(solver, { 1, 5 }, 1.0f, 0.5f, { N / 2.0f, 2.5f, 0 });

    for (int i = 0; i < M; i++)
        new Rigid(solver, { 2, 1 }, 1.0f, 0.5f, { 0, i * 2.0f + 8.0f, 0 });
}

static void sceneCards(Solver* solver)
{
    solver->clear();
    new Rigid(solver, { 80.0f, 4.0f }, 0.0f, 0.7f, { 0.0f, -2.0f, 0.0f });

    float cardHeight = 0.2f * 2.0f;
    float cardThickness = 0.001f * 2.0f;

    float angle0 = 25.0f * 3.14159f / 180.0f;
    float angle1 = -25.0f * 3.14159f / 180.0f;
    float angle2 = 0.5f * 3.14159f;

    int Nb = 5;
    float z0 = 0.0f;
    float y = cardHeight * 0.5f - 0.02f;
    while (Nb)
    {
        float z = z0;
        for (int i = 0; i < Nb; i++)
        {
            if (i != Nb - 1)
            {
                new Rigid(solver, { cardThickness, cardHeight }, 1.0f, 0.7f, { z + 0.25f, y + cardHeight * 0.5f - 0.02f, angle2 });
            }

            new Rigid(solver, { cardThickness, cardHeight }, 1.0f, 0.7f, { z, y, angle1 });

            z += 0.175f;

            new Rigid(solver, { cardThickness, cardHeight }, 1.0f, 0.7f, { z, y, angle0 });

            z += 0.175f;
        }
        y += cardHeight - 0.04f;
        z0 += 0.175f;
        Nb--;
    }
}


static void (*scenes[])(Solver*) =
{
    sceneEmpty,
    sceneGround,
    sceneDynamicFriction,
    sceneStaticFriction,
    scenePyramid,
    sceneCards,
    sceneRope,
    sceneHeavyRope,
    sceneHangingRope,
    sceneSpring,
    sceneSpringsRatio,
    sceneStack,
    sceneStackRatio,
    sceneRod,
    sceneSoftBody,
    sceneJointGrid,
    sceneNet,
    sceneMotor,
    sceneFracture
};

static const char* sceneNames[] = {
    "Empty",
    "Ground",
    "Dynamic Friction",
    "Static Friction",
    "Pyramid",
    "Cards",
    "Rope",
    "Heavy Rope",
    "Hanging Rope",
    "Spring",
    "Spring Ratio",
    "Stack",
    "Stack Ratio",
    "Rod",
    "Soft Body",
    "Joint Grid",
    "Net",
    "Motor",
    "Fracture"
};

static const int sceneCount = 19;
