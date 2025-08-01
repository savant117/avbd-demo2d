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

Motor::Motor(Solver* solver, Rigid* bodyA, Rigid* bodyB, float speed, float maxTorque)
    : Force(solver, bodyA, bodyB), speed(speed)
{
    fmax[0] = maxTorque;
    fmin[0] = -maxTorque;
}

void Motor::computeConstraint(float alpha)
{
    // Compute delta angular position between the two bodies
    float dAngleA = (bodyA ? (bodyA->position.z - bodyA->initial.z) : 0.0f);
    float dAngleB = bodyB->position.z - bodyB->initial.z;
    float deltaAngle = dAngleA - dAngleB;

    // Constraint tries to reach desired angular speed
    C[0] = deltaAngle - speed * solver->dt;
}

void Motor::computeDerivatives(Rigid* body)
{
    // Compute the first and second derivatives for the desired body
    if (body == bodyA)
    {
        J[0] = { 0.0f, 0.0f, 1.0f };
        H[0] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    }
    else
    {
        J[0] = { 0.0f, 0.0f, -1.0f };
        H[0] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    }
}
