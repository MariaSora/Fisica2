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

	wheel.color = Black;

	for(int i = 0; i < vehicle->getNumWheels(); ++i)
	{
		wheel.radius = info.wheels[0].radius;
		wheel.height = info.wheels[0].width;

		vehicle->updateWheelTransform(i);
		vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(&wheel.transform);

		wheel.Render();
	}

	CreateCarCube(vec3(info.chassis_size.x, info.chassis_size.y, info.chassis_size.z), { info.chassis_offset.x, info.chassis_offset.y, info.chassis_offset.z }, Blue).Render();
	CreateCarCube(vec3(3.0f, 0.3, 1.0f), { 0, 0.5f,3.0f }, Blue).Render(); 
	CreateCarCube(vec3(3.0f, 1.2, 1.5f), { 0, 1.0f,-1.7f }, Blue).Render(); 
	CreateCarCube(vec3(3.5f, 0.2, 0.2f), { 0, 0.9f,1.5f }, Black).Render(); //Retrovisores

	//Esto es como la cabina
	CreateCarCube(vec3(1.2f, 0.8f, 2.0f), { 0.0f, 1.5f,-0.5f }, White).Render(); 
	CreateCarCube(vec3(0.9f, 0.4f, 1.8f), { 0, 1.2f, 0 }, Blue).Render(); 
	
	//Parte de atras
	CreateCarCube(vec3(0.1f, 1.0f, 0.8f), { -0.8f, 1.5f,-2.0f }, Blue).Render();  
	CreateCarCube(vec3(0.1f, 1.0f, 0.8f), { 0.8f, 1.5f,-2.0f }, Blue).Render();  
	CreateCarCube(vec3(3.0f, 0.1f, 0.8f), { 0.0f, 2.0f,-2.0f }, Blue).Render();
	
	CreateCarCube(vec3(0.5f, 0.2f, 1.0f), { 0, 0.5f, -3.0f }, Black).Render();
	CreateCarCube(vec3(1.0f, 0.2f, 0.5f), { 0, 0.3f, -3.0f }, Black).Render(); 
	
	CreateCarCube(vec3(0.1f, 0.2f, 2.0f), { 1.2f, 1.2f, 0.5 }, Green).Render();
	CreateCarCube(vec3(0.1f, 0.2f, 2.0f), { -1.2f, 1.2f, 0.5 }, Green).Render();

	//Luceeees
	CreateCarCube(vec3(0.2f, 0.2f, 0.2f), { 1.2f, 0.8f, 2.5f }, White).Render();
	CreateCarCube(vec3(0.2f, 0.2f, 0.2f), { -1.2f, 0.8f, 2.5f }, White).Render();
	CreateCarCube(vec3(0.2f, 0.2f, 0.2f), { 1.2f, 0.8f, -2.5f }, Red).Render();
	CreateCarCube(vec3(0.2f, 0.2f, 0.2f), { -1.2f, 0.8f, -2.5f }, Red).Render();

	
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