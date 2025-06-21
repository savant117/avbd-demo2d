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

Joint::Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB, float2 rA, float2 rB, float3 stiffness, float motor, float fracture)
	: Force(solver, bodyA, bodyB), rA(rA), rB(rB)
{
	this->stiffness[0] = stiffness.x;
	this->stiffness[1] = stiffness.y;
	this->stiffness[2] = stiffness.z;
	this->motor[2] = motor;
	this->fmax[2] = fracture;
	this->fmin[2] = -fracture;
	this->fracture[2] = fracture;
	this->restAngle = (bodyA ? bodyA->position.z : 0.0f) - bodyB->position.z;
	this->torqueArm = lengthSq((bodyA ? bodyA->size : float2{ 0, 0 }) + bodyB->size);
}

bool Joint::initialize()
{
	C0.xy() = (bodyA ? transform(bodyA->position, rA) : rA) - transform(bodyB->position, rB);
	C0.z = ((bodyA ? bodyA->position.z : 0) - bodyB->position.z - restAngle) * torqueArm;
	return stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0;
}

void Joint::computeConstraint(float alpha)
{
	float3 Cn;
	Cn.xy() = (bodyA ? transform(bodyA->position, rA) : rA) - transform(bodyB->position, rB);
	Cn.z = ((bodyA ? bodyA->position.z : 0) - bodyB->position.z - restAngle) * torqueArm;

	for (int i = 0; i < rows(); i++)
	{
		if (isinf(stiffness[i]))
			C[i] = Cn[i] - C0[i] * alpha;
		else
			C[i] = Cn[i];
	}
}

void Joint::computeDerivatives(Rigid* body)
{
	if (body == bodyA)
	{
		float2 r = rotate(bodyA->position.z, rA);
		J[0] = { 1.0f, 0.0f, -r.y };
		J[1] = { 0.0f, 1.0f, r.x };
		J[2] = { 0.0f, 0.0f, torqueArm };
		H[0] = { 0, 0, 0, 0, 0, 0, 0, 0, -r.x };
		H[1] = { 0, 0, 0, 0, 0, 0, 0, 0, -r.y };
		H[2] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	}
	else
	{
		float2 r = rotate(bodyB->position.z, rB);
		J[0] = { -1.0f, 0.0f, r.y };
		J[1] = { 0.0f, -1.0f, -r.x };
		J[2] = { 0.0f, 0.0f, -torqueArm };
		H[0] = { 0, 0, 0, 0, 0, 0, 0, 0, r.x };
		H[1] = { 0, 0, 0, 0, 0, 0, 0, 0, r.y };
		H[2] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	}
}

void Joint::draw() const
{
	float2 v0 = bodyA ? transform(bodyA->position, rA) : rA;
	float2 v1 = transform(bodyB->position, rB);

	glColor3f(0.75f, 0.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex2f(v0.x, v0.y);
	glVertex2f(v1.x, v1.y);
	glEnd();
}
