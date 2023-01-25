#include "Test.hpp"

#include <Utility/Timer.hpp>
#include <Physics.hpp>
#include <RigidBody/ForceGenerators/ForceGeneratorsRB.hpp>
#include <MassAggregate/Constraints/HardConstraints.hpp>
#include <MassAggregate/ForceGenerators/ForceGenerators.hpp>
#include <MassAggregate/ParticleManager.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <IAnimatedMeshSceneNode.h>
#include <ISceneNode.h>
#include <irrlicht.h>

#include <Windows.h>

//=============================================================================
//TYPEDEF
//=============================================================================
using RigidBody = Ocacho::Physics::RigidBody::RigidBody;

//Irrlitch
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

//=============================================================================
//GLOBAL VARIABLES
//=============================================================================
Ocacho::Timer timer;
float deltaTime{};

std::array<RigidBody, 1> bodiesTest1{};

bool isBoosting{ false };
float currentVelocity{ 15.f };

//Irrlitch
irr::IrrlichtDevice* device;
irr::video::IVideoDriver* driver;
irr::scene::ISceneManager* smgr;
DirtyEvent receiver;
std::array<ISceneNode*, 1> nodesTest1 {};

//=============================================================================
//METHODS
//=============================================================================

void init()
{
	bodiesTest1[0].acceleration.y = -20.f;


  	device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1270, 720), 16, false, false, false, &receiver);
 
  	driver = device->getVideoDriver();
  	smgr = device->getSceneManager();
 
  	if (!device)
  		return;
 
  	smgr->addCameraSceneNode(0, vector3df(0,10,-30), vector3df(0,0,0));

	nodesTest1[0] = smgr->addSphereSceneNode(1.0f);
}

//=============================================================================
//=============================================================================

void checkInput()
{
	bodiesTest1[0].velocity.z = 0;
	bodiesTest1[0].velocity.x = 0;

	auto state = receiver.IsKeyDown(irr::KEY_KEY_W);
	if (state)
	{
		bodiesTest1[0].velocity.z = currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_S);
	if (state)
	{
		bodiesTest1[0].velocity.z = -currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_A);
	if (state)
	{
		bodiesTest1[0].velocity.x = -currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_D);
	if (state)
	{
		bodiesTest1[0].velocity.x = currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_SPACE);
	if (state)
	{
		if (bodiesTest1[0].velocity.y < 0.1f && bodiesTest1[0].velocity.y > -0.1f)
			bodiesTest1[0].velocity.y = 30.f;
	}
}

//=============================================================================
//=============================================================================

void updateNodes()
{
	for (uint8_t i = 0; i < nodesTest1.size(); ++i)
	{
		nodesTest1[i]->setPosition(irr::core::vector3d(bodiesTest1[i].position.x, bodiesTest1[i].position.y, bodiesTest1[i].position.z));
	}
}

//=============================================================================
//=============================================================================

int main()
{
	init();

	timer.start();

	while(device->run())
	{
		deltaTime = timer.ellapsedSeconds();

		if (deltaTime >= 0.016f)
		{
			timer.start();

			checkInput();

			for (uint8_t i = 0; i < bodiesTest1.size(); ++i)
			{
				Ocacho::Physics::RigidBody::IntegrateRigidBody(bodiesTest1[i], deltaTime);
			}

			if (bodiesTest1[0].position.y < 0.f)
			{
				bodiesTest1[0].position.y = 0.f;
				bodiesTest1[0].velocity.y = 0.f;
			}

			updateNodes();

			std::cout << "posicion: " << bodiesTest1[0].position << "\n";

			driver->beginScene(true, true, SColor(255, 100, 101, 140));

			smgr->drawAll();
			driver->endScene();

			if (receiver.IsKeyDown(irr::KEY_ESCAPE))
				break;
		}
	}

    device->drop();

	return 0;
}

//=============================================================================
//=============================================================================

int testParticle()
{
  	//Cosas de Irrlicht
  	irr::IrrlichtDevice* device;
  	irr::video::IVideoDriver* driver;
  	irr::scene::ISceneManager* smgr;
 
  	irr::core::line3d<irr::f32> lines[8];
  	std::vector<irr::scene::ISceneNode*> nodes;
 
  	nodes.reserve(5);
 
  	device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1270, 720), 16, false, false, false, &receiver);
 
  	driver = device->getVideoDriver();
  	smgr = device->getSceneManager();
 
  	if (!device)
  		return 0;
 
  	smgr->addCameraSceneNode(0, vector3df(0,0,50), vector3df(0,0,0));
  	float x, y, z;
  		
  	for(unsigned i=0; i<5; ++i)
  	{
  		nodes.emplace_back(smgr->addSphereSceneNode(1.0f));
  	}
  		
  	//Cosas de fisicas
  	using part = Ocacho::Physics::MassAggregate::Particle;
  	using cableHC = Ocacho::Physics::MassAggregate::Cable;
  	using rodHC = Ocacho::Physics::MassAggregate::Rod;
  	using gfGen = Ocacho::Physics::MassAggregate::GravityForceGenerator;
  	using dragGen = Ocacho::Physics::MassAggregate::DragForceGenerator;
  		
  	auto p1 = std::make_unique<part>();
  	auto p2 = std::make_unique<part>();
  	auto p3 = std::make_unique<part>();
  	auto p4 = std::make_unique<part>();
  	auto p5 = std::make_unique<part>();
  	p1->mass = 10000000000.0f;
  	p1->inverseMass = 1 / p1->mass;
  	p1->position.x = 10.0f;
  	p1->position.z = 5.0f;
 
  	p2->position.x = 20.0f;
  	p3->position.x = 10.0f;
  	p4->position.x = 0.0f;
  	p5->position.x = -10.0f;
 
  	auto gravGen = std::make_unique<gfGen>();
  	auto drGen = std::make_unique<dragGen>();
 
  	cableHC cable = cableHC(10.0f);
  	cable.particles_[0] = p1.get();
  	cable.particles_[1] = p2.get();
 
  	cableHC cable2 = cableHC(11.0f);
  	cable2.particles_[0] = p1.get();
  	cable2.particles_[1] = p3.get();
 
  	cableHC cable3 = cableHC(11.0f);
  	cable3.particles_[0] = p1.get();
  	cable3.particles_[1] = p4.get();
 
  	cableHC cable4 = cableHC(11.0f);
  	cable4.particles_[0] = p1.get();
  	cable4.particles_[1] = p5.get();
 
  	rodHC rod = rodHC(6.0f);
  	rod.particles_[0] = p2.get();
  	rod.particles_[1] = p3.get();
 
  	rodHC rod2 = rodHC(6.0f);
  	rod2.particles_[0] = p3.get();
  	rod2.particles_[1] = p4.get();
 
  	rodHC rod3 = rodHC(6.0f);
  	rod3.particles_[0] = p4.get();
  	rod3.particles_[1] = p5.get();
 
  	rodHC rod4 = rodHC(6.0f);
  	rod4.particles_[0] = p5.get();
  	rod4.particles_[1] = p2.get();
 
  	rodHC rod5 = rodHC(8.49f);
  	rod5.particles_[0] = p2.get();
  	rod5.particles_[1] = p4.get();
 
  	rodHC rod6 = rodHC(8.49f);
  	rod6.particles_[0] = p3.get();
  	rod6.particles_[1] = p5.get();
 
  	using contactGenList = Ocacho::Physics::MassAggregate::Typelist< cableHC, rodHC >;
  	using forceList = Ocacho::Physics::MassAggregate::Typelist< gfGen, dragGen>;
 
  	using partMan = Ocacho::Physics::MassAggregate::ParticleManager<forceList, contactGenList>;
 
  	partMan particleManager = partMan(15, 15);
 
  	//Adding the particles to the manager
  	particleManager.addParticle(*p1.get());
  	particleManager.addParticle(*p2.get());
  	particleManager.addParticle(*p3.get());
  	particleManager.addParticle(*p4.get());
  	particleManager.addParticle(*p5.get());
 
  	//Adding the contact generators
  	particleManager.addContactGenerator(cable);
  	particleManager.addContactGenerator(cable2);
  	particleManager.addContactGenerator(cable3);
  	particleManager.addContactGenerator(cable4);
 
  	particleManager.addContactGenerator(rod);
  	particleManager.addContactGenerator(rod2);
  	particleManager.addContactGenerator(rod3);
  	particleManager.addContactGenerator(rod4);
  	particleManager.addContactGenerator(rod5);
  	particleManager.addContactGenerator(rod6);
 
  	particleManager.addForceRegistration(*p2.get(), *gravGen);
  	particleManager.addForceRegistration(*p3.get(), *gravGen);
  	particleManager.addForceRegistration(*p4.get(), *gravGen);
  	particleManager.addForceRegistration(*p5.get(), *gravGen);

    Ocacho::Timer timer;
    float deltaTime;
 
  	while(device->run())
  	{
  		if(timer.ellapsedSeconds() >= 0.016f)
  		{
  			particleManager.reset();
 
  			deltaTime = timer.ellapsedSeconds();
  			timer.start();
 
  			particleManager.runPhysics(deltaTime);
 
  			{
  				//Actualizacion de las posiciones de los nodos y lineas de la escena
  				nodes[0]->setPosition(irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z));
  				nodes[1]->setPosition(irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z));
  				nodes[2]->setPosition(irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z));
  				nodes[3]->setPosition(irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z));
  				nodes[4]->setPosition(irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z));
  					
  				lines[0].start = irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z);
  				lines[0].end = irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z);
 
  				lines[1].start = irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z);
  				lines[1].end = irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z);
 
  				lines[2].start = irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z);
  				lines[2].end = irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z);
 
  				lines[3].start = irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z);
  				lines[3].end = irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z);
 
  				lines[4].start = irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z);
  				lines[4].end = irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z);
 
  				lines[5].start = irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z);
  				lines[5].end = irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z);
 
  				lines[6].start = irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z);
  				lines[6].end = irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z);
 
  				lines[7].start = irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z);
  				lines[7].end = irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z);
  			}
 
  			driver->beginScene(true, true, SColor(255,100,101,140));
 
  			driver->setTransform(video::ETS_WORLD, core::IdentityMatrix);
  			for(unsigned i=0; i<8; ++i)
  				driver->draw3DLine(lines[i].start, lines[i].end, video::SColor(45, 45, 45, 256));
 
  			smgr->drawAll();
 
  			driver->endScene();
 
  			x=y=z=0;
 
  			if(receiver.IsKeyDown(irr::KEY_KEY_W))
  			{
  				z = -1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_S))
  			{
  				z = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_A))
  			{
  				x = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_D))
  			{
  				x = -1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_SPACE))
  			{
  				y = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_LSHIFT))
  			{
  				y = -1;
  			}
 
  			p1->position.x += x;
  			p1->position.y += y;
  			p1->position.z += z;
 
  			if(receiver.IsKeyDown(irr::KEY_KEY_Q))
  				break;
  		}
  	}
 
  	device->drop();

    return 0;
}

//=============================================================================
//=============================================================================

int testSprings()
{
  	//Cosas de Irrlicht
  	irr::IrrlichtDevice* device;
  	irr::video::IVideoDriver* driver;
  	irr::scene::ISceneManager* smgr;
 
  	irr::core::line3d<irr::f32> lines[8];
  	std::vector<irr::scene::ISceneNode*> nodes;
 
  	nodes.reserve(5);
 
  	device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1270, 720), 16, false, false, false, &receiver);
 
  	driver = device->getVideoDriver();
  	smgr = device->getSceneManager();
 
  	if (!device)
  		return 0;
 
  	smgr->addCameraSceneNode(0, vector3df(0,0,50), vector3df(0,0,0));
  	float x, y, z;
  		
  	for(unsigned i=0; i<5; ++i)
  	{
  		nodes.emplace_back(smgr->addSphereSceneNode(1.0f));
  	}
  		
  	//Cosas de fisicas
  	using part = Ocacho::Physics::MassAggregate::Particle;
  	using spring = Ocacho::Physics::MassAggregate::AnchoredSpringForceGenerator;
  	using cableHC = Ocacho::Physics::MassAggregate::Cable;
  	using rodHC = Ocacho::Physics::MassAggregate::Rod;
  	using gfGen = Ocacho::Physics::MassAggregate::GravityForceGenerator;
  	using dragGen = Ocacho::Physics::MassAggregate::DragForceGenerator;
  		
  	auto p1 = std::make_unique<part>();
  	auto p2 = std::make_unique<part>();
  	auto p3 = std::make_unique<part>();
  	auto p4 = std::make_unique<part>();
  	auto p5 = std::make_unique<part>();
  		
  	p1->position.x = 10.0f;
  	p1->position.z = 5.0f;
  	p2->position.x = 20.0f;
  	p3->position.x = 10.0f;
  	p4->position.x = 0.0f;
  	p5->position.x = -10.0f;
 
  	auto gravGen = std::make_unique<gfGen>();
  	auto drGen = std::make_unique<dragGen>();
 
  	Ocacho::Timer timer;
  	float deltaTime;
 
  	using contactGenList = Ocacho::Physics::MassAggregate::Typelist< cableHC, rodHC >;
  	using forceList = Ocacho::Physics::MassAggregate::Typelist< gfGen, dragGen, spring>;
 
  	using partMan = Ocacho::Physics::MassAggregate::ParticleManager<forceList, contactGenList>;
 
  	partMan particleManager = partMan(15, 15);
 
  	//Adding the particles to the manager
  	particleManager.addParticle(*p1.get());
  	particleManager.addParticle(*p2.get());
  	particleManager.addParticle(*p3.get());
  	particleManager.addParticle(*p4.get());
  	particleManager.addParticle(*p5.get());
 
  	//Creating the spring force generators
  	auto spr = std::make_unique<spring>(5.f, 10.f, Ocacho::Vector3(0.f, 0.f, 0.f));
  	particleManager.addForceRegistration(*p1.get(), *spr);
  	particleManager.addForceRegistration(*p2.get(), *spr);
  	particleManager.addForceRegistration(*p3.get(), *spr);
  	particleManager.addForceRegistration(*p4.get(), *spr);
  	particleManager.addForceRegistration(*p5.get(), *spr);
 
  	particleManager.addForceRegistration(*p2.get(), *gravGen);
  	particleManager.addForceRegistration(*p3.get(), *gravGen);
  	particleManager.addForceRegistration(*p4.get(), *gravGen);
  	particleManager.addForceRegistration(*p5.get(), *gravGen);
 
  	while(device->run())
  	{
  		if(timer.ellapsedSeconds() >= 0.016f)
  		{
  			particleManager.reset();
 
  			deltaTime = timer.ellapsedSeconds();
  			timer.start();
 
  			particleManager.runPhysics(deltaTime);
 
  			{
  				//Actualizacion de las posiciones de los nodos y lineas de la escena
  				nodes[0]->setPosition(irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z));
  				nodes[1]->setPosition(irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z));
  				nodes[2]->setPosition(irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z));
  				nodes[3]->setPosition(irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z));
  				nodes[4]->setPosition(irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z));
  					
  				lines[0].start = irr::core::vector3df(0.f, 0.f, 0.f);
  				lines[0].end = irr::core::vector3df(p2->position.x, p2->position.y, p2->position.z);
 
  				lines[1].start = irr::core::vector3df(0.f, 0.f, 0.f);
  				lines[1].end = irr::core::vector3df(p3->position.x, p3->position.y, p3->position.z);
 
  				lines[2].start = irr::core::vector3df(0.f, 0.f, 0.f);
  				lines[2].end = irr::core::vector3df(p4->position.x, p4->position.y, p4->position.z);
 
  				lines[3].start = irr::core::vector3df(0.f, 0.f, 0.f);
  				lines[3].end = irr::core::vector3df(p5->position.x, p5->position.y, p5->position.z);
 
  				lines[4].start = irr::core::vector3df(0.f, 0.f, 0.f);
  				lines[4].end = irr::core::vector3df(p1->position.x, p1->position.y, p1->position.z);
  			}
 
  			driver->beginScene(true, true, SColor(255,100,101,140));
 
  			driver->setTransform(video::ETS_WORLD, core::IdentityMatrix);
  			for(unsigned i=0; i<5; ++i)
  				driver->draw3DLine(lines[i].start, lines[i].end, video::SColor(45, 45, 45, 256));
 
  			smgr->drawAll();
 
  			driver->endScene();
 
  			x=y=z=0;
 
  			if(receiver.IsKeyDown(irr::KEY_KEY_W))
  			{
  				z = -1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_S))
  			{
  				z = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_A))
  			{
  				x = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_KEY_D))
  			{
  				x = -1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_SPACE))
  			{
  				y = 1;
  			}
  			if(receiver.IsKeyDown(irr::KEY_LSHIFT))
  			{
  				y = -1;
  			}
 
  			if(receiver.IsKeyDown(irr::KEY_KEY_Q))
  				break;
  		}
  	}
 
  	device->drop();

    return 0;
}