/*
Title: SAT-3D (AABB)
File Name: Main.cpp
Copyright © 2015
Original authors: Brockton Roth
Modified by: Parth Contractor
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a Separating Axis Theorem test. (Sometimes just called Separating Axis Test.) This is in 3D.
Contains two cubes, one that is stationary and one that is moving. They are bounded by AABBs (Axis-Aligned
Bounding Boxes) and when these AABBs collide the moving object "bounces" on the x axis (because that is the
only direction the object is moving). The algorithm will detect any axis of collision, but will not output
the axis that was collided (because it doesn't know). Thus, we assume x and hardcode in the x axis bounce.
There is a physics timestep such that every update runs at the same delta time, regardless of how fast or
slow the computer is running. The cubes would be the exact same as their AABBs, if not for the fact that
they are constantly rotating. Since the AABB recalculates every frame, on it will always encapsulate the
largest form of the object on all 3 axes. If you wish to see it without rotation, comment out the rotate
lines (obj1->Rotate, obj2->Rotate).
*/

#include "GLIncludes.h"
#include "GLRender.h"
#include "GameObject.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// Variables for FPS and Physics Timestep calculations.
int frame = 0;
double time = 0;
double timebase = 0;
double accumulator = 0.0;
int fps = 0;
double FPSTime = 0.0;
double physicsStep = 0.012; // This is the number of milliseconds we intend for the physics to update.

bool antiStuck = false;

// Reference to the window object being created by GLFW.
GLFWwindow* window;

// Returns the 8 points for a given AABB. Largely just a helper function for this specific implementation.
std::vector<glm::vec3> GetPoints(AABB obj)
{
	std::vector<glm::vec3> points;

	// Left face
	// minx, miny, minz
	// minx, maxy, minz
	// minx, maxy, maxz
	// minx, miny, maxz
	points.push_back(glm::vec3(obj.min.x, obj.min.y, obj.min.z));
	points.push_back(glm::vec3(obj.min.x, obj.max.y, obj.min.z));
	points.push_back(glm::vec3(obj.min.x, obj.max.y, obj.max.z));
	points.push_back(glm::vec3(obj.min.x, obj.min.y, obj.max.z));

	// Right face
	// maxx, maxy, maxz
	// maxx, maxy, minz
	// maxx, miny, minz
	// maxx, miny, maxz
	points.push_back(glm::vec3(obj.max.x, obj.max.y, obj.max.z));
	points.push_back(glm::vec3(obj.max.x, obj.max.y, obj.min.z));
	points.push_back(glm::vec3(obj.max.x, obj.min.y, obj.min.z));
	points.push_back(glm::vec3(obj.max.x, obj.min.y, obj.max.z));

	return points;
}

// Returns the normals for an AABB. Also contains commented implementation for returning the normals of an arbitrary set of points.
std::vector<glm::vec3> GetNormals(AABB obj)
{
	// Since AABB is a cube, there should be 6 normals
	// But it's aligned on the axes, so there are only 3 we need to worry about, X, Y, and Z.
	std::vector<glm::vec3> normals;

	normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));

	// Below is a potential implementation for calculating the normals of a given set of points, assuming the points are ordered in groups of 3 where each 3 points is 
	// a triangle on a face. You could then go through and remove duplicate normals as well.
	//for (int i = 0; i < points.size(); i += 3)
	//{
	//	glm::vec3 U(points[i + 1] - points[i]);
	//	glm::vec3 V(points[i + 2] - points[i]);

	//	normals.push_back(glm::normalize(glm::vec3((U.y * V.z) - (U.z * V.y), (U.z * V.x) - (U.x * V.z), (U.x * V.y) - (U.y * V.x))));

	//	/*currentNormal.x = (U.y * V.z) - (U.z * V.y);
	//	currentNormal.y = (U.z * V.x) - (U.x * V.z);
	//	currentNormal.z = (U.x * V.y) - (U.y * V.x);*/

	//}

	return normals;
}

// Gets the minimum and maximum projections for a set of points given an axis to project them onto.
// Will output the values into the min and max variables passed into the function.
void GetMinMax(std::vector<glm::vec3> points, glm::vec3 axis, float& min, float&max)
{
	// To get a projection along a vector, you take the dot product of the vertex (in vector form) with the axis vector.
	// Since we're looking for both a minimum and a maximum, we start with the first point, and go from there.
	min = glm::dot(points[0], axis);
	max = min;

	// Looping through the rest of the points.
	for (int i = 1; i < points.size(); i++)
	{
		// Find the projection for the current point.
		float currProj = glm::dot(points[i], axis);

		// If this projection is smaller than our minimum projection, the minimum becomes this.
		if (min > currProj)
		{
			min = currProj;
		}

		// If this projection is larger than our maximum projection, the maximum becomes this.
		if (currProj > max)
		{
			max = currProj;
		}
	}
}

// Returns true if there is no collision, and false if there is a collision.
bool TestSAT(AABB a, AABB b)
{
	// Since these are AABB collisions, we can easily determine the normals. That said, this was left this way for understanding and future implementation.
	// The first step is to get the normals for the two colliding objects, as these will be the axes on which we test for collisions.
	std::vector<glm::vec3> aNormals = GetNormals(a);
	std::vector<glm::vec3> bNormals = GetNormals(b);
	
	// A quick method that exists for getting the points of the AABB. In a regular implementation, we might instead pass in the actual points to this algorithm, skipping
	// this step.
	std::vector<glm::vec3> aPoints = GetPoints(a);
	std::vector<glm::vec3> bPoints = GetPoints(b);

	// This boolean gets returned, and will be true if there is no collision.
	bool isSeparated = false;

	// For each normal
	for (int i = 0; i < aNormals.size(); i++)
	{
		// Get the Min and Max projections for each object along the normal.
		float aMin, aMax;
		GetMinMax(aPoints, aNormals[i], aMin, aMax);

		float bMin, bMax;
		GetMinMax(bPoints, aNormals[i], bMin, bMax);

		// If the maximum projection of one of the objects is less than the minimum projection of the other object, then we can determine that there is a separation 
		// along this axis. Thus, we set isSeparated to true and break out of the for loop.
		isSeparated = aMax < bMin || bMax < aMin;
		if (isSeparated) break;
	}

	// This only runs if we still haven't proven that there is a separation between the two objects.
	// SAT is an optimistic algorithm in that it will stop the moment it determines there isn't a collision, and as such the less collisions there are the faster it will run.
	if (!isSeparated)
	{
		// Loop through the normals for the second object.
		// Note that since this is an AABB, the normals will be the same as the previous object. Again, this is left for future implementation and understanding.
		// The process below is exactly the same as above, only with object b's normals instead of object a.
		for (int i = 0; i < bNormals.size(); i++)
		{
			float aMin, aMax;
			GetMinMax(aPoints, bNormals[i], aMin, aMax);

			float bMin, bMax;
			GetMinMax(bPoints, bNormals[i], bMin, bMax);

			isSeparated = aMax < bMin || bMax < aMin;
			if (isSeparated) break;
		}
	}

	// At this point, isSeparated has been tested against each normal. If it has been set to true, then there is a separation. If it is false, that means none of the axes 
	// were separated, and there is a collision.
	return isSeparated;
}

// This runs once every physics timestep.
void update(float dt)
{
#pragma region Boundaries
	// This section just checks to make sure the object stays within a certain boundary. This is not really collision detection.
	glm::vec3 tempPos = obj2->GetPosition();
	
	if (fabsf(tempPos.x) > 1.35f)
	{
		glm::vec3 tempVel = obj2->GetVelocity();

		// "Bounce" the velocity along the axis that was over-extended.
		obj2->SetVelocity(glm::vec3(-1.0f * tempVel.x, tempVel.y, tempVel.z));
	}
	if (fabsf(tempPos.y) > 0.8f)
	{
		glm::vec3 tempVel = obj2->GetVelocity();
		obj2->SetVelocity(glm::vec3(tempVel.x, -1.0f * tempVel.y, tempVel.z));
	}
	if (fabsf(tempPos.z) > 1.0f)
	{
		glm::vec3 tempVel = obj2->GetVelocity();
		obj2->SetVelocity(glm::vec3(tempVel.x, tempVel.y, -1.0f * tempVel.z));
	}
#pragma endregion Boundaries section just bounces the object so it does not fly off the side of the screen infinitely.

	// Rotate the objects. This helps illustrate how the AABB recalculates as an object's orientation changes.
	 obj1->Rotate(glm::vec3(glm::radians(1.0f), glm::radians(1.0f), glm::radians(0.0f)));
	 obj2->Rotate(glm::vec3(glm::radians(1.0f), glm::radians(1.0f), glm::radians(0.0f)));

	// Re-calculate the Axis-Aligned Bounding Box for your object.
	// We do this because if the object's orientation changes, we should update the bounding box as well.
	// Be warned: For some objects this can actually cause a collision to be missed, so be careful.
	// (This is because we determine the time of the collision based on the AABB, but if the AABB changes significantly, the time of collision can change between frames,
	// and if that lines up just right you'll miss the collision altogether.)
	obj1->CalculateAABB();
	obj2->CalculateAABB();

	// Pass in our two objects to the SAT test, if it returns false then they are colliding because there is no separating axis between them.
	if (!TestSAT(obj2->GetAABB(), obj1->GetAABB()) && !antiStuck)
	{
		glm::vec3 velocity = obj2->GetVelocity();
		
		// Reverse the velocity in the x direction
		// This is the "bounce" effect, only we don't actually know the axis of collision from the test. Instead, we assume it because the object is only moving in the x 
		// direction.
		velocity.x *= -1;

		obj2->SetVelocity(velocity);

		// This variable exists to allow an extra frame/update to pass when they collide so that the objects do not get stuck as a result of tunneling and 
		// recalculating the AABB.
		antiStuck = true;
	}
	else if (antiStuck)
	{
		antiStuck = false;
	}
	
	// Update the objects based on their velocities.
	obj1->Update(dt);
	obj2->Update(dt);

	// Update your MVP matrices based on the objects' transforms.
	MVP = PV * *obj1->GetTransform();
	MVP2 = PV * *obj2->GetTransform();
}

// This runs once every frame to determine the FPS and how often to call update based on the physics step.
void checkTime()
{
	// Get the current time.
	time = glfwGetTime();

	// Get the time since we last ran an update.
	double dt = time - timebase;

	// If more time has passed than our physics timestep.
	if (dt > physicsStep)
	{
		// Calculate FPS: Take the number of frames (frame) since the last time we calculated FPS, and divide by the amount of time that has passed since the 
		// last time we calculated FPS (time - FPSTime).
		if (time - FPSTime > 1.0)
		{
			fps = frame / (time - FPSTime);

			FPSTime = time; // Now we set FPSTime = time, so that we have a reference for when we calculated the FPS
			
			frame = 0; // Reset our frame counter to 0, to mark that 0 frames have passed since we calculated FPS (since we literally just did it)

			std::string s = "FPS: " + std::to_string(fps); // This just creates a string that looks like "FPS: 60" or however much.

			glfwSetWindowTitle(window, s.c_str()); // This will set the window title to that string, displaying the FPS as the window title.
		}

		timebase = time; // Set timebase = time so we have a reference for when we ran the last physics timestep.

		// Limit dt so that we if we experience any sort of delay in processing power or the window is resizing/moving or anything, it doesn't update a bunch of times while the player can't see.
		// This will limit it to a .25 seconds.
		if (dt > 0.25)
		{
			dt = 0.25;
		}

		// The accumulator is here so that we can track the amount of time that needs to be updated based on dt, but not actually update at dt intervals and instead use our physicsStep.
		accumulator += dt;

		// Run a while loop, that runs update(physicsStep) until the accumulator no longer has any time left in it (or the time left is less than physicsStep, at which point it save that 
		// leftover time and use it in the next checkTime() call.
		while (accumulator >= physicsStep)
		{
			update(physicsStep);

			accumulator -= physicsStep;
		}
	}
}



int main(int argc, char **argv)
{
	// Initializes the GLFW library
	glfwInit();

	// Creates a window given (width, height, title, monitorPtr, windowPtr).
	// Don't worry about the last two, as they have to do with controlling which monitor to display on and having a reference to other windows. Leaving them as nullptr is fine.
	window = glfwCreateWindow(800, 600, "SAT 3D AABB Collision", nullptr, nullptr);

	// Makes the OpenGL context current for the created window.
	glfwMakeContextCurrent(window);
	
	// Sets the number of screen updates to wait before swapping the buffers.
	// Setting this to zero will disable VSync, which allows us to actually get a read on our FPS. Otherwise we'd be consistently getting 60FPS or lower, 
	// since it would match our FPS to the screen refresh rate.
	// Set to 1 to enable VSync.
	glfwSwapInterval(0);

	// Initializes most things needed before the main loop
	init();

	// Calculate the Axis-Aligned Bounding Box for your object.
	obj1->CalculateAABB();
	obj2->CalculateAABB();

	// Enter the main loop.
	while (!glfwWindowShouldClose(window))
	{
		// Call to checkTime() which will determine how to go about updating via a set physics timestep as well as calculating FPS.
		checkTime();

		// Call the render function.
		renderScene();

		// Swaps the back buffer to the front buffer
		// Remember, you're rendering to the back buffer, then once rendering is complete, you're moving the back buffer to the front so it can be displayed.
		glfwSwapBuffers(window);

		// Add one to our frame counter, since we've successfully 
		frame++;

		// Checks to see if any events are pending and then processes them.
		glfwPollEvents();
	}

	cleanup();

	return 0;
}