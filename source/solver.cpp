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

Solver::Solver()
    : bodies(0), forces(0)
{
    defaultParams();
}

Solver::~Solver()
{
    clear();
}

Rigid* Solver::pick(float2 at, float2& local)
{
    // Find which body is at the given point
    for (Rigid* body = bodies; body != 0; body = body->next)
    {
        float2x2 Rt = rotation(-body->position.z);
        local = Rt * (at - body->position.xy());
        if (local.x >= -body->size.x * 0.5f && local.x <= body->size.x * 0.5f &&
            local.y >= -body->size.y * 0.5f && local.y <= body->size.y * 0.5f)
            return body;
    }
    return 0;
}

void Solver::clear()
{
    while (forces)
        delete forces;

    while (bodies)
        delete bodies;
}

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = -10.0f;
    iterations = 10;

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 100000.0f;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.99f;

    // Post stabilization applies an extra iteration to fix positional error.
    // This removes the need for the alpha parameter, which can make tuning a little easier.
    postStabilize = true;
}

void Solver::step()
{
    // Perform broadphase collision detection
    // This is a naive O(n^2) approach, but it is sufficient for small numbers of bodies in this sample.
    for (Rigid* bodyA = bodies; bodyA != 0; bodyA = bodyA->next)
    {
        for (Rigid* bodyB = bodyA->next; bodyB != 0; bodyB = bodyB->next)
        {
            float2 dp = bodyA->position.xy() - bodyB->position.xy();
            float r = bodyA->radius + bodyB->radius;
            if (dot(dp, dp) <= r * r && !bodyA->constrainedTo(bodyB))
                new Manifold(this, bodyA, bodyB);
        }
    }

    // Initialize and warmstart forces
    for (Force* force = forces; force != 0;)
    {
        // Initialization can including caching anything that is constant over the step
        if (!force->initialize())
        {
            // Force has returned false meaning it is inactive, so remove it from the solver
            Force* next = force->next;
            delete force;
            force = next;
        }
        else
        {
            for (int i = 0; i < force->rows(); i++)
            {
                if (postStabilize)
                {
                    // With post stabilization, we can reuse the full lambda from the previous step,
                    // and only need to reduce the penalty parameters
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                }
                else
                {
                    // Warmstart the dual variables and penalty parameters (Eq. 19)
                    // Penalty is safely clamped to a minimum and maximum value
                    force->lambda[i] = force->lambda[i] * alpha * gamma;
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                }

                // If it's not a hard constraint, we don't let the penalty exceed the material stiffness
                force->penalty[i] = min(force->penalty[i], force->stiffness[i]);
            }

            force = force->next;
        }
    }

    // Initialize and warmstart bodies (ie primal variables)
    for (Rigid* body = bodies; body != 0; body = body->next)
    {
        // Don't let bodies rotate too fast
        body->velocity.z = clamp(body->velocity.z, -50.0f, 50.0f);

        // Compute inertial position (Eq 2)
        body->inertial = body->position + body->velocity * dt;
        if (body->mass > 0)
            body->inertial += float3{ 0, gravity, 0 } * (dt * dt);

        // Adaptive warmstart (See original VBD paper)
        float3 accel = (body->velocity - body->prevVelocity) / dt;
        float accelExt = accel.y * sign(gravity);
        float accelWeight = clamp(accelExt / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight)) accelWeight = 0.0f;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body->initial = body->position;
        body->position = body->position + body->velocity * dt + float3{ 0, gravity, 0 } * (accelWeight * dt * dt);
    }

    // Main solver loop
    // If using post stabilization, we'll use one extra iteration for the stabilization
    int totalIterations = iterations + (postStabilize ? 1 : 0);

    for (int it = 0; it < totalIterations; it++)
    {
        // If using post stabilization, either remove all or none of the pre-existing constraint error
        float currentAlpha = alpha;
        if (postStabilize)
            currentAlpha = it < iterations ? 1.0f : 0.0f;

        // Primal update
        for (Rigid* body = bodies; body != 0; body = body->next)
        {
            // Skip static / kinematic bodies
            if (body->mass <= 0)
                continue;

            // Initialize left and right hand sides of the linear system (Eqs. 5, 6)
            float3x3 M = diagonal(body->mass, body->mass, body->moment);
            float3x3 lhs = M / (dt * dt);
            float3 rhs = M / (dt * dt) * (body->position - body->inertial);

            // Iterate over all forces acting on the body
            for (Force* force = body->forces; force != 0; force = (force->bodyA == body) ? force->nextA : force->nextB)
            {
                // Compute constraint and its derivatives
                force->computeConstraint(currentAlpha);
                force->computeDerivatives(body);

                for (int i = 0; i < force->rows(); i++)
                {
                    // Use lambda as 0 if it's not a hard constraint
                    float lambda = isinf(force->stiffness[i]) ? force->lambda[i] : 0.0f;

                    // Compute the clamped force magnitude (Sec 3.2)
                    float f = clamp(force->penalty[i] * force->C[i] + lambda, force->fmin[i], force->fmax[i]);

                    // Compute the diagonally lumped geometric stiffness term (Sec 3.5)
                    float3x3 G = diagonal(length(force->H[i].col(0)), length(force->H[i].col(1)), length(force->H[i].col(2))) * abs(f);

                    // Accumulate force (Eq. 13) and hessian (Eq. 17)
                    rhs += force->J[i] * f;
                    lhs += outer(force->J[i], force->J[i] * force->penalty[i]) + G;
                }
            }

            // Solve the SPD linear system using LDL and apply the update (Eq. 4)
            body->position -= solve(lhs, rhs);
        }

        // Dual update, only for non stabilized iterations in the case of post stabilization
        // If doing more than one post stabilization iteration, we can still do a dual update,
        // but make sure not to persist the penalty or lambda updates done during the stabilization iterations for the next frame.
        if (it < iterations)
        {
            for (Force* force = forces; force != 0; force = force->next)
            {
                // Compute constraint
                force->computeConstraint(currentAlpha);

                for (int i = 0; i < force->rows(); i++)
                {
                    // Use lambda as 0 if it's not a hard constraint
                    float lambda = isinf(force->stiffness[i]) ? force->lambda[i] : 0.0f;

                    // Update lambda (Eq 11)
                    force->lambda[i] = clamp(force->penalty[i] * force->C[i] + lambda, force->fmin[i], force->fmax[i]);

                    // Disable the force if it has exceeded its fracture threshold
                    if (fabsf(force->lambda[i]) >= force->fracture[i])
                        force->disable();

                    // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                    if (force->lambda[i] > force->fmin[i] && force->lambda[i] < force->fmax[i])
                        force->penalty[i] = min(force->penalty[i] + beta * abs(force->C[i]), min(PENALTY_MAX, force->stiffness[i]));
                }
            }
        }

        // If we are are the final iteration before post stabilization, compute velocities (BDF1)
        if (it == iterations - 1)
        {
            for (Rigid* body = bodies; body != 0; body = body->next)
            {
                body->prevVelocity = body->velocity;
                if (body->mass > 0)
                    body->velocity = (body->position - body->initial) / dt;
            }
        }
    }
}

void Solver::draw()
{
    for (Rigid* body = bodies; body != 0; body = body->next)
        body->draw();
    for (Force* force = forces; force != 0; force = force->next)
        force->draw();
}
