#include "PhysVehicle3D.h"
#include "Primitive.h"
#include "Bullet/include/btBulletDynamicsCommon.h"

// ----------------------------------------------------------------------------
VehicleInfo::~VehicleInfo()
{
	//if(wheels != NULL)
		//delete wheels;
}

// ----------------------------------------------------------------------------
PhysVehicle3D::PhysVehicle3D(btRigidBody* body, btRaycastVehicle* vehicle, const VehicleInfo& info) : PhysBody3D(body), vehicle(vehicle), info(info)
{
}

// ----------------------------------------------------------------------------
PhysVehicle3D::~PhysVehicle3D()
{
	delete vehicle;
}

// ----------------------------------------------------------------------------
void PhysVehicle3D::Render()
{
	Cylinder wheel;

	wheel.color = Orange;

	for(int i = 0; i < vehicle->getNumWheels(); ++i)
	{
		wheel.radius = info.wheels[0].radius;
		wheel.height = info.wheels[0].width;

		vehicle->updateWheelTransform(i);
		vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(&wheel.transform);

		wheel.Render();
	}

	/*Cube chassis(info.chassis_size.x, info.chassis_size.y, info.chassis_size.z);
	vehicle->getChassisWorldTransform().getOpenGLMatrix(&chassis.transform);
	btQuaternion q = vehicle->getChassisWorldTransform().getRotation();
	btVector3 offset(info.chassis_offset.x, info.chassis_offset.y, info.chassis_offset.z);
	offset = offset.rotate(q.getAxis(), q.getAngle());

	chassis.transform.M[12] += offset.getX();
	chassis.transform.M[13] += offset.getY();
	chassis.transform.M[14] += offset.getZ();


	chassis.Render();*/

	CreateCarCube(vec3(info.chassis_size.x, info.chassis_size.y, info.chassis_size.z), { info.chassis_offset.x, info.chassis_offset.y, info.chassis_offset.z }, Red).Render();
	CreateCarCube(vec3(1.0f, 0.9, 2.0f), { 0, 0.7f,3.0f }, Red).Render();
	CreateCarCube(vec3(3.0f, 0.3, 1.2f), { 0, 0.7f,4.5f }, Red).Render();
	CreateCarCube(vec3(1.2f, 0.8f, 2.0f), { 0.0f, 1.5f,-0.5f }, White).Render();
	CreateCarCube(vec3(0.1f, 1.0f, 0.8f), { -0.8f, 1.5f,-2.0f }, Red).Render(); 
	CreateCarCube(vec3(0.1f, 1.0f, 0.8f), { 0.8f, 1.5f,-2.0f }, Red).Render(); 
	CreateCarCube(vec3(3.0f, 0.1f, 0.8f), {0.0f, 2.0f,-2.0f}, Red).Render();
}

Cube PhysVehicle3D::CreateCarCube(const vec3 size, const vec3 position, const Color color)
{
	Cube part(size.x, size.y, size.z);

	vehicle->getChassisWorldTransform().getOpenGLMatrix(&part.transform);

	btQuaternion q = vehicle->getChassisWorldTransform().getRotation();
	btVector3 m_offset(position.x, position.y, position.z);

	m_offset = m_offset.rotate(q.getAxis(), q.getAngle());

	part.transform.M[12] += m_offset.getX();
	part.transform.M[13] += m_offset.getY();
	part.transform.M[14] += m_offset.getZ();

	part.color = color;

	return part;
}

// ----------------------------------------------------------------------------
void PhysVehicle3D::ApplyEngineForce(float force)
{
	for(int i = 0; i < vehicle->getNumWheels(); ++i)
	{
		if(info.wheels[i].drive == true)
		{
			vehicle->applyEngineForce(force, i);
		}
	}
}

// ----------------------------------------------------------------------------
void PhysVehicle3D::Brake(float force)
{
	for(int i = 0; i < vehicle->getNumWheels(); ++i)
	{
		if(info.wheels[i].brake == true)
		{
			vehicle->setBrake(force, i);
		}
	}
}

// ----------------------------------------------------------------------------
void PhysVehicle3D::Turn(float degrees)
{
	for(int i = 0; i < vehicle->getNumWheels(); ++i)
	{
		if(info.wheels[i].steering == true)
		{
			vehicle->setSteeringValue(degrees, i);
		}
	}
}

// ----------------------------------------------------------------------------
float PhysVehicle3D::GetKmh() const
{
	return vehicle->getCurrentSpeedKmHour();
}