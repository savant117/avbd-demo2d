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

#include "solver.h"

Rigid::Rigid(Solver* solver, float2 size, float density, float friction, float3 position, float3 velocity)
    : solver(solver), forces(0), next(0), position(position), velocity(velocity), prevVelocity(velocity), size(size), friction(friction)
{
    // Add to linked list
    next = solver->bodies;
    solver->bodies = this;

    // Compute mass properties and bounding radius
    mass = size.x * size.y * density;
    moment = mass * dot(size, size) / 12.0f;
    radius = length(size * 0.5f);
}

Rigid::~Rigid()
{
    // Remove from linked list
    Rigid** p = &solver->bodies;
    while (*p != this)
        p = &(*p)->next;
    *p = next;
}

bool Rigid::constrainedTo(Rigid* other) const
{
    // Check if this body is constrained to the other body
    for (Force* f = forces; f != 0; f = f->next)
        if ((f->bodyA == this && f->bodyB == other) || (f->bodyA == other && f->bodyB == this))
            return true;
    return false;
}

void Rigid::draw()
{
    float2x2 R = rotation(position.z);
    float2 v0 = R * float2{ -size.x * 0.5f, -size.y * 0.5f } + position.xy();
    float2 v1 = R * float2{ size.x * 0.5f, -size.y * 0.5f } + position.xy();
    float2 v2 = R * float2{ size.x * 0.5f, size.y * 0.5f } + position.xy();
    float2 v3 = R * float2{ -size.x * 0.5f, size.y * 0.5f } + position.xy();

    glColor3f(0.6f, 0.6f, 0.6f);
    glBegin(GL_QUADS);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glVertex2f(v2.x, v2.y);
    glVertex2f(v3.x, v3.y);
    glEnd();

    glColor3f(0, 0, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glVertex2f(v2.x, v2.y);
    glVertex2f(v3.x, v3.y);
    glEnd();
}
