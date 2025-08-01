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

#include "maths.h"

#define MAX_ROWS 4                    // Most number of rows an individual constraint can have
#define PENALTY_MIN 1.0f              // Minimum penalty parameter
#define PENALTY_MAX 1000000000.0f     // Maximum penalty parameter
#define COLLISION_MARGIN 0.0005f      // Margin for collision detection to avoid flickering contacts
#define STICK_THRESH 0.01f            // Position threshold for sticking contacts (ie static friction)
#define SHOW_CONTACTS true            // Whether to show contacts in the debug draw

struct Rigid;
struct Force;
struct Manifold;
struct Solver;

// Holds all the state for a single rigid body that is needed by AVBD
struct Rigid
{
    Solver* solver;
    Force* forces;
    Rigid* next;
    float3 position;
    float3 initial;
    float3 inertial;
    float3 velocity;
    float3 prevVelocity;
    float2 size;
    float mass;
    float moment;
    float friction;
    float radius;

    Rigid(Solver* solver, float2 size, float density, float friction, float3 position, float3 velocity = float3{ 0, 0, 0 });
    ~Rigid();

    bool constrainedTo(Rigid* other) const;
    void draw();
};

// Holds all user defined and derived constraint parameters, and provides a common interface for all forces.
struct Force
{
    Solver* solver;
    Rigid* bodyA;
    Rigid* bodyB;
    Force* nextA;
    Force* nextB;
    Force* next;

    float3 J[MAX_ROWS];
    float3x3 H[MAX_ROWS];
    float C[MAX_ROWS];
    float fmin[MAX_ROWS];
    float fmax[MAX_ROWS];
    float stiffness[MAX_ROWS];
    float fracture[MAX_ROWS];
    float penalty[MAX_ROWS];
    float lambda[MAX_ROWS];

    Force(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    virtual ~Force();

    void disable();

    virtual int rows() const = 0;
    virtual bool initialize() = 0;
    virtual void computeConstraint(float alpha) = 0;
    virtual void computeDerivatives(Rigid* body) = 0;
    virtual void draw() const {}
};

// Revolute joint + angle constraint between two rigid bodies, with optional fracture
struct Joint : Force
{
    float2 rA, rB;
    float3 C0;
    float torqueArm;
    float restAngle;

    Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB, float2 rA, float2 rB, float3 stiffness = float3{ INFINITY, INFINITY, INFINITY },
        float fracture = INFINITY);

    int rows() const override { return 3; }

    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;
};

// Standard spring force
struct Spring : Force
{
    float2 rA, rB;
    float rest;

    Spring(Solver* solver, Rigid* bodyA, Rigid* bodyB, float2 rA, float2 rB, float stiffness, float rest = -1);

    int rows() const override { return 1; }

    bool initialize() override { return true; }
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;
};

// Force which has no physical effect, but is used to ignore collisions between two bodies
struct IgnoreCollision : Force
{
    IgnoreCollision(Solver* solver, Rigid* bodyA, Rigid* bodyB)
        : Force(solver, bodyA, bodyB) {}

    int rows() const override { return 0; }

    bool initialize() override { return true; }
    void computeConstraint(float alpha) override {}
    void computeDerivatives(Rigid* body) override {}
    void draw() const override {}
};

// Motor force which applies a torque to two rigid bodies to achieve a desired angular speed
struct Motor : Force
{
    float speed;

    Motor(Solver* solver, Rigid* bodyA, Rigid* bodyB, float speed, float maxTorque);

    int rows() const override { return 1; }

    bool initialize() override { return true; }
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override {}
};

// Collision manifold between two rigid bodies, which contains up to two frictional contact points
struct Manifold : Force
{
    // Used to track contact features between frames
    union FeaturePair
    {
        struct Edges
        {
            char inEdge1;
            char outEdge1;
            char inEdge2;
            char outEdge2;
        } e;
        int value;
    };

    // Contact point information for a single contact
    struct Contact
    {
        FeaturePair feature;
        float2 rA;
        float2 rB;
        float2 normal;

        float3 JAn, JBn, JAt, JBt;
        float2 C0;
        bool stick;
    };

    Contact contacts[2];
    int numContacts;
    float friction;

    Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);

    int rows() const override { return numContacts * 2; }

    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;

    static int collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts);
};

// Core solver class which holds all the rigid bodies and forces, and has logic to step the simulation forward in time
struct Solver
{
    float dt;           // Timestep
    float gravity;      // Gravity
    int iterations;     // Solver iterations

    float alpha;        // Stabilization parameter
    float beta;         // Penalty ramping parameter
    float gamma;        // Warmstarting decay parameter

    bool postStabilize; // Whether to apply post-stabilization to the system

    Rigid* bodies;
    Force* forces;

    Solver();
    ~Solver();

    Rigid* pick(float2 at, float2& local);
    void clear();
    void defaultParams();
    void step();
    void draw();
};
