/**
* @file Simulation/Simulation.cpp
* Implementation of class Simulation
* @author Colin Graf
*/

#include <cmath>
#include <algorithm>

#include "Platform/Assert.h"
#include "Platform/System.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Body.h"
#include "Simulation/Geometries/Geometry.h"
#include "Parser/Parser.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"
#ifdef MULTI_THREADING
#include <thread>
#endif

Simulation* Simulation::simulation = 0;

Simulation::Simulation() : scene(0), physicalWorld(0), rootSpace(0), staticSpace(0), movableSpace(0),
  currentFrameRate(0),
  simulationStep(0), simulatedTime(0), collisions(0), contactPoints(0),
  contactGroup(0),
  lastFrameRateComputationTime(0), lastFrameRateComputationStep(0)
{
  ASSERT(simulation == 0);
  simulation = this;
}

Simulation::~Simulation()
{
  for(std::list<Element*>::const_iterator iter = elements.begin(), end = elements.end(); iter != end; ++iter)
    delete *iter;

  if(contactGroup)
    dJointGroupDestroy(contactGroup);
  if(rootSpace)
    dSpaceDestroy(rootSpace);
  if(physicalWorld)
  {
#ifdef MULTI_THREADING
    dThreadingImplementationShutdownProcessing(threading);
    dThreadingThreadPoolWaitIdleState(pool);
    dThreadingFreeThreadPool(pool);
    dWorldSetStepThreadingImplementation(physicalWorld, nullptr, nullptr);
    dThreadingFreeImplementation(threading);
#endif
    dWorldDestroy(physicalWorld);
    dCloseODE();
  }

  ASSERT(simulation == this);
  simulation = 0;
}

bool Simulation::loadFile(const std::string& filename, std::list<std::string>& errors)
{
  ASSERT(scene == 0);

  Parser parser;
  if(!parser.parse(filename, errors))
    return false;

  ASSERT(scene);

  dInitODE();
  physicalWorld = dWorldCreate();
  rootSpace = dHashSpaceCreate(0);
  staticSpace = dHashSpaceCreate(rootSpace);
  movableSpace = dHashSpaceCreate(rootSpace);
  contactGroup = dJointGroupCreate(0);

  dWorldSetGravity(physicalWorld, 0, 0, scene->gravity);
  if(scene->erp != -1.f)
    dWorldSetERP(physicalWorld, scene->erp);
  if(scene->cfm != -1.f)
    dWorldSetCFM(physicalWorld, scene->cfm);
  if(scene->quickSolverIterations != -1)
    dWorldSetQuickStepNumIterations(physicalWorld, scene->quickSolverIterations);
#ifdef MULTI_THREADING
  threading = dThreadingAllocateMultiThreadedImplementation();
  pool = dThreadingAllocateThreadPool(std::thread::hardware_concurrency(), 0, dAllocateFlagBasicData, nullptr);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  dWorldSetStepThreadingImplementation(physicalWorld, dThreadingImplementationGetFunctions(threading), threading);
#endif

  scene->createPhysics();

  renderer.init();

  return true;
}

void Simulation::doSimulationStep()
{
  ++simulationStep;
  simulatedTime += scene->stepLength;

  scene->updateActuators();

  collisions = contactPoints = 0;

  dSpaceCollide2(reinterpret_cast<dGeomID>(staticSpace), reinterpret_cast<dGeomID>(movableSpace), this, reinterpret_cast<dNearCallback*>(&staticCollisionWithSpaceCallback));
  if(scene->detectBodyCollisions)
    dSpaceCollide(movableSpace, this, reinterpret_cast<dNearCallback*>(&staticCollisionSpaceWithSpaceCallback));

  if(scene->useQuickSolver && (simulationStep % scene->quickSolverSkip) == 0)
    dWorldQuickStep(physicalWorld, scene->stepLength);
  else
    dWorldStep(physicalWorld, scene->stepLength);
  dJointGroupEmpty(contactGroup);

  updateFrameRate();
}

void Simulation::staticCollisionWithSpaceCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(!dGeomIsSpace(geomId1));
  ASSERT(dGeomIsSpace(geomId2));
  dSpaceCollide2(geomId1, geomId2, simulation, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
}

void Simulation::staticCollisionSpaceWithSpaceCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(dGeomIsSpace(geomId1));
  ASSERT(dGeomIsSpace(geomId2));
  dSpaceCollide2(geomId1, geomId2, simulation, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
}

void Simulation::staticCollisionCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(!dGeomIsSpace(geomId1));
  ASSERT(!dGeomIsSpace(geomId2));

#ifndef NDEBUG
  {
    dBodyID bodyId1 = dGeomGetBody(geomId1);
    dBodyID bodyId2 = dGeomGetBody(geomId2);
    ASSERT(bodyId1 || bodyId2);

    Body* body1 = bodyId1 ? static_cast<Body*>(dBodyGetData(bodyId1)) : 0;
    Body* body2 = bodyId2 ? static_cast<Body*>(dBodyGetData(bodyId2)) : 0;
    ASSERT(!body1 || !body2 || body1->rootBody != body2->rootBody);
  }
#endif

  dContact contact[32];
  int collisions = dCollide(geomId1, geomId2, 32, &contact[0].geom, sizeof(dContact));
  if(collisions <= 0)
    return;

  Geometry* geometry1 = static_cast<Geometry*>(dGeomGetData(geomId1));
  Geometry* geometry2 = static_cast<Geometry*>(dGeomGetData(geomId2));

  if(geometry1->collisionCallbacks && !geometry2->immaterial)
  {
    for(std::list<SimRobotCore2::CollisionCallback*>::iterator i = geometry1->collisionCallbacks->begin(), end = geometry1->collisionCallbacks->end(); i != end; ++i)
      (*i)->collided(*geometry1, *geometry2);
    if(geometry1->immaterial)
      return;
  }
  if(geometry2->collisionCallbacks && !geometry1->immaterial)
  {
    for(std::list<SimRobotCore2::CollisionCallback*>::iterator i = geometry2->collisionCallbacks->begin(), end = geometry2->collisionCallbacks->end(); i != end; ++i)
      (*i)->collided(*geometry2, *geometry1);
    if(geometry2->immaterial)
      return;
  }

  dBodyID bodyId1 = dGeomGetBody(geomId1);
  dBodyID bodyId2 = dGeomGetBody(geomId2);
  ASSERT(bodyId1 || bodyId2);

  float friction = 1.f;
  if(geometry1->material && geometry2->material)
  {
    if(!geometry1->material->getFriction(*geometry2->material, friction))
      friction = 1.f;

    float rollingFriction;
    if(bodyId1)
      switch(dGeomGetClass(geomId1))
      {
        case dSphereClass:
        case dCCylinderClass:
        case dCylinderClass:
          if(geometry1->material->getRollingFriction(*geometry2->material, rollingFriction))
          {
            dBodySetAngularDamping(bodyId1, 0.2f);
            Vector3f linearVel;
            ODETools::convertVector(dBodyGetLinearVel(bodyId1), linearVel);
            linearVel -= linearVel.normalized(std::min(linearVel.norm(), rollingFriction * simulation->scene->stepLength));
            dBodySetLinearVel(bodyId1, linearVel.x(), linearVel.y(), linearVel.z());
          }
          break;
      }
    if(bodyId2)
      switch(dGeomGetClass(geomId2))
      {
        case dSphereClass:
        case dCCylinderClass:
        case dCylinderClass:
          if(geometry2->material->getRollingFriction(*geometry1->material, rollingFriction))
          {
            dBodySetAngularDamping(bodyId2, 0.2f);
            Vector3f linearVel;
            ODETools::convertVector(dBodyGetLinearVel(bodyId2), linearVel);
            linearVel -= linearVel.normalized(std::min(linearVel.norm(), rollingFriction * simulation->scene->stepLength));
            dBodySetLinearVel(bodyId2, linearVel.x(), linearVel.y(), linearVel.z());
          }
          break;
      }
  }

  for(dContact* cont = contact, * end = contact + collisions; cont < end; ++cont)
  {
    cont->surface.mode = simulation->scene->contactMode | dContactApprox1;
    cont->surface.mu = friction;

    /*
    cont->surface.bounce = 0.f;
    cont->surface.bounce_vel = 0.001f;
    cont->surface.slip1 = 0.f;
    cont->surface.slip2 = 0.f;
    */
    cont->surface.soft_erp = simulation->scene->contactSoftERP;
    cont->surface.soft_cfm = simulation->scene->contactSoftCFM;

    dJointID c = dJointCreateContact(simulation->physicalWorld, simulation->contactGroup, cont);
    ASSERT(bodyId1 == dGeomGetBody(cont->geom.g1));
    ASSERT(bodyId2 == dGeomGetBody(cont->geom.g2));
    dJointAttach(c, bodyId1, bodyId2);
  }
  ++simulation->collisions;
  simulation->contactPoints += collisions;
}

void Simulation::updateFrameRate()
{
  unsigned int currentTime = System::getTime();
  unsigned int timeDiff = currentTime - lastFrameRateComputationTime;
  //Only update frame rate once in two seconds
  if(timeDiff > 2000)
  {
    float frameRate = float(simulationStep - lastFrameRateComputationStep) / (float(timeDiff) * 0.001f);
    currentFrameRate = int(frameRate + 0.5f);
    lastFrameRateComputationTime = currentTime;
    lastFrameRateComputationStep = simulationStep;
  }
}

void Simulation::registerObjects()
{
  scene->fullName = scene->name.c_str();
  CoreModule::application->registerObject(*CoreModule::module, *scene, 0);
  scene->registerObjects();
}
