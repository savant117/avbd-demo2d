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

#include <cmath>

using namespace std;

// Math types

struct float2
{
    float x, y;

    float& operator[](int i) { return ((float*)this)[i]; }
    const float& operator[](int i) const { return ((float*)this)[i]; }
};

struct float3
{
    float x, y, z;

    float2& xy() { return *(float2*)this; }
    const float2& xy() const { return *(float2*)this; }

    float& operator[](int i) { return ((float*)this)[i]; }
    const float& operator[](int i) const { return ((float*)this)[i]; }
};

struct float2x2
{
    float2 row[2];

    float2& operator[](int i) { return row[i]; }
    const float2& operator[](int i) const { return row[i]; }

    float2 col(int i) const { return float2 { row[0][i], row[1][i] }; }
};

struct float3x3
{
    float3 row[3];

    float3& operator[](int i) { return row[i]; }
    const float3& operator[](int i) const { return row[i]; }

    float3 col(int i) const { return float3{ row[0][i], row[1][i], row[2][i] }; }
};

// float2 operators

float dot(float2 a, float2 b);

inline float2 operator+=(float2& a, float2 b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

inline float2 operator-=(float2& a, float2 b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

inline float2 operator-(float2 v)
{
    return { -v.x, -v.y };
}

inline float2 operator+(float2 a, float2 b)
{
    return { a.x + b.x, a.y + b.y };
}

inline float2 operator-(float2 a, float2 b)
{
    return { a.x - b.x, a.y - b.y };
}

inline float2 operator*(float2 a, float b)
{
    return { a.x * b, a.y * b };
}

inline float2 operator/(float2 a, float b)
{
    return { a.x / b, a.y / b };
}

inline float2 operator*(float2x2 a, float2 b)
{
    return { dot(a[0], b), dot(a[1], b) };
}

// float3 operators

float dot(float3 a, float3 b);

inline float3& operator+=(float3& a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline float3& operator-=(float3& a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline float3 operator-(float3 v)
{
    return { -v.x, -v.y, -v.z };
}

inline float3 operator+(float3 a, float3 b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline float3 operator-(float3 a, float3 b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

inline float3 operator*(float3 a, float b)
{
    return { a.x * b, a.y * b, a.z * b };
}

inline float3 operator/(float3 a, float b)
{
    return { a.x / b, a.y / b, a.z / b };
}

inline float3 operator*(float3x3 a, float3 b)
{
    return { dot(a[0], b), dot(a[1], b), dot(a[2], b) };
}

// float2x2 operators

inline float2x2 operator+(float2x2 a, float2x2 b)
{
    return { a[0] + b[0], a[1] + b[1] };
}

inline float2x2 operator-(float2x2 a, float2x2 b)
{
    return { a[0] - b[0], a[1] - b[1] };
}

inline float2x2 operator*(float2x2 a, float b)
{
    return { a[0] * b, a[1] * b };
}

inline float2x2 operator/(float2x2 a, float b)
{
    return { a[0] / b, a[1] / b };
}

inline float2x2 operator*(float2x2 a, float2x2 b)
{
    return { 
        float2 { dot(a.row[0], b.col(0)), dot(a.row[0], b.col(1)) },
        float2 { dot(a.row[1], b.col(0)), dot(a.row[1], b.col(1)) } 
    };
}

// float3x3 operators

inline float3x3& operator+=(float3x3& a, float3x3 b)
{
    a[0] += b[0];
    a[1] += b[1];
    a[2] += b[2];
    return a;
}

inline float3x3 operator+(float3x3 a, float3x3 b)
{
    return { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

inline float3x3 operator-(float3x3 a, float3x3 b)
{
    return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

inline float3x3 operator*(float3x3 a, float b)
{
    return { a[0] * b, a[1] * b, a[2] * b };
}

inline float3x3 operator/(float3x3 a, float b)
{
    return { a[0] / b, a[1] / b, a[2] / b };
}

// Math functions

inline float sign(float x)
{
    return x < 0 ? -1.0f : x > 0 ? 1.0f : 0.0f;
}

inline float min(float a, float b)
{
    return a < b ? a : b;
}

inline float max(float a, float b)
{
    return a > b ? a : b;
}

inline float clamp(float x, float a, float b)
{
    return max(a, min(b, x));
}

inline float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}

inline float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float lengthSq(float2 v)
{
    return dot(v, v);
}

inline float length(float2 v)
{
    return sqrtf(dot(v, v));
}

inline float lengthSq(float3 v)
{
    return dot(v, v);
}

inline float length(float3 v)
{
    return sqrtf(dot(v, v));
}

inline float cross(float2 a, float2 b)
{
    return a.x * b.y - a.y * b.x;
}

inline float2x2 outer(float2 a, float2 b)
{
    return { b * a.x, b * a.y };
}

inline float3x3 outer(float3 a, float3 b)
{
    return { b * a.x, b * a.y, b * a.z };
}

inline float2 abs(float2 v)
{
    return { fabsf(v.x), fabsf(v.y) };
}

inline float2x2 abs(float2x2 a)
{
    return { abs(a[0]), abs(a[1]) };
}

inline float2x2 transpose(float2x2 a)
{
    return { float2 { a[0][0], a[1][0] }, float2 {a[0][1], a[1][1] }};
}

inline float2x2 rotation(float angle)
{
    float c = cos(angle);
    float s = sin(angle);
    return { c, -s, s, c };
}

inline float3x3 diagonal(float m00, float m11, float m22)
{
    return float3x3 {
        m00, 0, 0, 
        0, m11, 0,
        0, 0, m22
    };
}

inline float2 transform(float3 q, float2 v)
{
    return rotation(q.z) * v + q.xy();
}

inline float2 rotate(float angle, float2 v)
{
    return rotation(angle) * v;
}

inline float3 solve(float3x3 a, float3 b)
{
    // Compute LDL^T decomposition
    float D1 = a[0][0];
    float L21 = a[1][0] / a[0][0];
    float L31 = a[2][0] / a[0][0];
    float D2 = a[1][1] - L21 * L21 * D1;
    float L32 = (a[2][1] - L21 * L31 * D1) / D2;
    float D3 = a[2][2] - (L31 * L31 * D1 + L32 * L32 * D2);

    // Forward substitution: Solve Ly = b
    float y1 = b.x;
    float y2 = b.y - L21 * y1;
    float y3 = b.z - L31 * y1 - L32 * y2;

    // Diagonal solve: Solve Dz = y
    float z1 = y1 / D1;
    float z2 = y2 / D2;
    float z3 = y3 / D3;

    // Backward substitution: Solve L^T x = z
    float3 x;
    x[2] = z3;
    x[1] = z2 - L32 * x[2];
    x[0] = z1 - L21 * x[1] - L31 * x[2];

    return x;
}
