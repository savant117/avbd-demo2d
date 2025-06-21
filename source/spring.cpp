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

Spring::Spring(Solver* solver, Rigid* bodyA, Rigid* bodyB, float2 rA, float2 rB, float stiffness, float rest)
    : Force(solver, bodyA, bodyB), rA(rA), rB(rB), rest(rest)
{
    this->stiffness[0] = stiffness;
    if (this->rest < 0)
        this->rest = length(transform(bodyA->position, rA) - transform(bodyB->position, rB));
}

void Spring::computeConstraint(float alpha)
{
    C[0] = length(transform(bodyA->position, rA) - transform(bodyB->position, rB)) - rest;
}

void Spring::computeDerivatives(Rigid* body)
{
    float2x2 S = { 0, -1, 1, 0 };
    float2x2 I = { 1, 0, 0, 1 };

    float2 d = transform(bodyA->position, rA) - transform(bodyB->position, rB);
    float dlen2 = dot(d, d);
    if (dlen2 == 0)
        return;
    float dlen = sqrtf(dlen2);
    float2 n = d / dlen;
    float2x2 dxx = (I - outer(n, n) / dlen2) / dlen;

    if (body == bodyA)
    {
        float2 Sr = rotate(bodyA->position.z, S * rA);
        float2 r = rotate(bodyA->position.z, rA);
        float2 dxr = dxx * Sr;
        float drr = dot(Sr, dxr) - dot(n, r);

        J[0].xy() = n;
        J[0].z = dot(n, Sr);
        H[0] = {
            dxx.row[0].x, dxx.row[0].y, dxr.x,
            dxx.row[1].x, dxx.row[1].y, dxr.y,
            dxr.x,          dxr.y,        drr
        };
    }
    else
    {
        float2 Sr = rotate(bodyB->position.z, S * rB);
        float2 r = rotate(bodyB->position.z, rB);
        float2 dxr = dxx * -Sr;
        float drr = dot(Sr, dxr) + dot(n, r);

        J[0].xy() = -n;
        J[0].z = dot(n, -Sr);
        H[0] = {
            dxx.row[0].x, dxx.row[0].y, dxr.x,
            dxx.row[1].x, dxx.row[1].y, dxr.y,
            dxr.x,          dxr.y,        drr
        };
    }
}

void Spring::draw() const
{
    float2 v0 = transform(bodyA->position, rA);
    float2 v1 = transform(bodyB->position, rB);

    glColor3f(0.75f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glEnd();
}
