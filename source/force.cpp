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

Force::Force(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : solver(solver), bodyA(bodyA), bodyB(bodyB), nextA(0), nextB(0)
{
    // Add to solver linked list
    next = solver->forces;
    solver->forces = this;

    // Add to body linked lists
    if (bodyA)
    {
        nextA = bodyA->forces;
        bodyA->forces = this;
    }
    if (bodyB)
    {
        nextB = bodyB->forces;
        bodyB->forces = this;
    }

    // Set some reasonable defaults
    for (int i = 0; i < MAX_ROWS; i++)
    {
        J[i] = { 0, 0, 0 };
        H[i] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        C[i] = 0.0f;
        stiffness[i] = INFINITY;
        fmax[i] = INFINITY;
        fmin[i] = -INFINITY;
        fracture[i] = INFINITY;

        penalty[i] = 0.0f;
        lambda[i] = 0.0f;
    }
}


Force::~Force()
{
    // Remove from solver linked list
    Force** p = &solver->forces;
    while (*p != this)
        p = &(*p)->next;
    *p = next;

    // Remove from body linked lists
    if (bodyA)
    {
        p = &bodyA->forces;
        while (*p != this)
            p = (*p)->bodyA == bodyA ? &(*p)->nextA : &(*p)->nextB;
        *p = nextA;
    }

    if (bodyB)
    {
        p = &bodyB->forces;
        while (*p != this)
            p = (*p)->bodyA == bodyB ? &(*p)->nextA : &(*p)->nextB;
        *p = nextB;
    }
}

void Force::disable()
{
    // Disable this force by clearing the relavent fields
    for (int i = 0; i < MAX_ROWS; i++)
    {
        stiffness[i] = 0;
        penalty[i] = 0;
        lambda[i] = 0;
    }
}
