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

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB)
    : Force(solver, bodyA, bodyB), numContacts(0)
{
    fmax[0] = fmax[2] = 0.0f;
    fmin[0] = fmin[2] = -INFINITY;
}

bool Manifold::initialize()
{
    // Compute friction
    friction = sqrtf(bodyA->friction * bodyB->friction);

    // Store previous contact state
    Contact oldContacts[2] = { contacts[0], contacts[1] };
    float oldPenalty[4] = { penalty[0], penalty[1], penalty[2], penalty[3] };
    float oldLambda[4] = { lambda[0], lambda[1], lambda[2], lambda[3] };
    bool oldStick[2] = { contacts[0].stick, contacts[1].stick };
    int oldNumContacts = numContacts;

    // Compute new contacts
    numContacts = collide(bodyA, bodyB, contacts);

    // Merge old contact data with new contacts
    for (int i = 0; i < numContacts; i++)
    {
        penalty[i * 2 + 0] = penalty[i * 2 + 1] = 0.0f;
        lambda[i * 2 + 0] = lambda[i * 2 + 1] = 0.0f;

        for (int j = 0; j < oldNumContacts; j++)
        {
            if (contacts[i].feature.value == oldContacts[j].feature.value)
            {
                penalty[i * 2 + 0] = oldPenalty[j * 2 + 0];
                penalty[i * 2 + 1] = oldPenalty[j * 2 + 1];
                lambda[i * 2 + 0] = oldLambda[j * 2 + 0];
                lambda[i * 2 + 1] = oldLambda[j * 2 + 1];
                contacts[i].stick = oldStick[j];

                // If static friction in last frame, use the old contact points
                if (oldStick[j])
                {
                    contacts[i].rA = oldContacts[j].rA;
                    contacts[i].rB = oldContacts[j].rB;
                }
            }
        }
    }

    for (int i = 0; i < numContacts; i++)
    {
        // Compute the contact basis (Eq. 15)
        float2 normal = contacts[i].normal;
        float2 tangent = { normal.y, -normal.x };
        float2x2 basis = {
            normal.x, normal.y,
            tangent.x, tangent.y
        };

        float2 rAW = rotate(bodyA->position.z, contacts[i].rA);
        float2 rBW = rotate(bodyB->position.z, contacts[i].rB);

        // Precompute the constraint and derivatives at C(x-), since we use a truncated Taylor series for contacts (Sec 4).
        // Note that we discard the second order term, since it is insignificant for contacts
        contacts[i].JAn = { basis[0][0], basis[0][1], cross(rAW, normal) };
        contacts[i].JBn = { -basis[0][0], -basis[0][1], -cross(rBW, normal) };
        contacts[i].JAt = { basis[1][0], basis[1][1], cross(rAW, tangent) };
        contacts[i].JBt = { -basis[1][0], -basis[1][1], -cross(rBW, tangent) };

        contacts[i].C0 = basis * (bodyA->position.xy() + rAW - bodyB->position.xy() - rBW) + float2{ COLLISION_MARGIN, 0 };
    }

    return numContacts > 0;
}

void Manifold::computeConstraint(float alpha)
{
    for (int i = 0; i < numContacts; i++)
    {
        // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
        float3 dpA = bodyA->position - bodyA->initial;
        float3 dpB = bodyB->position - bodyB->initial;
        
        C[i * 2 + 0] = contacts[i].C0.x * (1 - alpha) + dot(contacts[i].JAn, dpA) + dot(contacts[i].JBn, dpB);
        C[i * 2 + 1] = contacts[i].C0.y * (1 - alpha) + dot(contacts[i].JAt, dpA) + dot(contacts[i].JBt, dpB);

        // Update the friction bounds using the latest lambda values
        float frictionBound = abs(lambda[i * 2 + 0]) * friction;
        fmax[i * 2 + 1] = frictionBound;
        fmin[i * 2 + 1] = -frictionBound;

        // Check if the contact is sticking, so that on the next frame we can use the old contact points for better static friction handling
        contacts[i].stick = abs(lambda[i * 2 + 1]) < frictionBound && abs(contacts[i].C0.y) < STICK_THRESH;
    }
}

void Manifold::computeDerivatives(Rigid* body)
{
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        if (body == bodyA)
        {
            J[i * 2 + 0] = contacts[i].JAn;
            J[i * 2 + 1] = contacts[i].JAt;
        }
        else
        {
            J[i * 2 + 0] = contacts[i].JBn;
            J[i * 2 + 1] = contacts[i].JBt;
        }
    }
}

void Manifold::draw() const
{
    if (!SHOW_CONTACTS)
        return;

    for (int i = 0; i < numContacts; i++)
    {
        float2 v0 = transform(bodyA->position, contacts[i].rA);
        float2 v1 = transform(bodyB->position, contacts[i].rB);

        glColor3f(0.75f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        glVertex2f(v0.x, v0.y);
        glVertex2f(v1.x, v1.y);
        glEnd();
    }
}
