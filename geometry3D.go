package main

import (
	"math"
	"fmt"
)

type Vec3 struct {
	X, Y, Z float64
}

// V returns a new 2D vector with the given coordinates
func V3(x, y, z float64) Vec3 {
	return Vec3{x, y, z}
}

// Unit returns a vector of length 1 facing the given angle
func Unit(angle float64) Vec3 {
	return Vec3{1, 0, 0}.Rotated(angle)
}

// Add returns the sum of vectors u and v
func (u Vec3) Add(v Vec3) Vec3 {
	return Vec3{u.X + v.X, u.Y + v.Y, u.Z + v.Z}
}

// Sub returns the difference betweeen vectors u and v
func (u Vec3) Sub(v Vec3) Vec3 {
	return Vec3{u.X - v.X, u.Y - v.Y, u.Z - v.Z}
}

// Scaled returns the vector u multiplied by c
func (u Vec3) Scaled(c float64) Vec3 {
	return Vec3{u.X * c, u.Y * c, u.Z * c}
}

// Len returns the length of the vector u
func (u Vec3) Len() float64 {
	return math.Hypot(math.Hypot(u.X, u.Y), u.Z)
}

// Angle returns the angle between the vector u and the x-axis. The result is in range [-Pi, Pi]
func (u Vec3) Angle() float64 {
	return math.Atan2(u.Y, u.X)
}

// Unit returns a vector of length 1 facing the direction of u (has the same angle)
func (u Vec3) Unit() Vec3 {
	if u.X == 0 && u.Y == 0 && u.Z == 0 {
		return Vec3{1, 0, 0}
	}
	return u.Scaled(1 / u.Len())
}

func (u Vec3) SetLen(c float64) Vec3 {
	if u.X == 0 && u.Y == 0 && u.Z == 0 {
		return Vec3{c, 0, 0}
	}
	return u.Scaled(c / u.Len())
}

// Rotated returns the vector u rotated by the given angle in radians
func (u Vec3) Rotated(angle float64) Vec3 {
	sin, cos := math.Sincos(angle)
	return Vec3{u.X*cos - u.Y*sin, u.X*sin + u.Y*cos, u.Z}
}

func (u Vec3) ReduceZ() Vec3 {
	return Vec3{u.X, u.Y, 0}
}

func (u Vec3) Print() {
	fmt.Println("vec X: ", u.X, "   Y:", u.Y, "   Z:", u.Z)
}
