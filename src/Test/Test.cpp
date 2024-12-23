#include "Test.hpp"

#include <Utility/Timer.hpp>
#include <Physics.hpp>

#include <RigidBody/ForceGenerators/ForceGeneratorsRB.hpp>
//#include <RigidBody/ForceGenerators/ForceGeneratorBaseRB.hpp>
#include <RigidBody/RigidBodyManager.hpp>
#include <Collisions/CollideFine.hpp>
#include <Collisions/Contacts.hpp>

#include <MassAggregate/Constraints/HardConstraints.hpp>
#include <MassAggregate/ForceGenerators/ForceGenerators.hpp>
#include <MassAggregate/ParticleManager.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <IAnimatedMeshSceneNode.h>
#include <ISceneNode.h>
#include <irrlicht.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <cmath>
#include <stdlib.h>
#include<chrono>

#pragma optimize("", off)

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

bool isBoosting{ false };
float currentVelocity{ 15.f };

//Irrlitch
irr::IrrlichtDevice* device;
irr::video::IVideoDriver* driver;
irr::scene::ISceneManager* smgr;
DirtyEvent receiver;

//=============================================================================
//METHODS
//=============================================================================

void checkInput(auto& entity)
{
	entity.velocity.z = 0;
	entity.velocity.x = 0;

	auto state = receiver.IsKeyDown(irr::KEY_KEY_W);
	if (state)
	{
		entity.velocity.z = currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_S);
	if (state)
	{
		entity.velocity.z = -currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_A);
	if (state)
	{
		entity.velocity.x = -currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_KEY_D);
	if (state)
	{
		entity.velocity.x = currentVelocity;
	}

	state = receiver.IsKeyDown(irr::KEY_SPACE);
	if (state)
	{
		if (entity.velocity.y < 0.1f && entity.velocity.y > -0.1f)
			entity.velocity.y = 30.f;
	}
}

//=============================================================================
// PARTICLES TEST
//=============================================================================

//=============================================================================
//TEMPLATES
//=============================================================================
template<typename TData>
class data_wrapper
{
public:
    data_wrapper() = default;
    auto& operator[](int i) noexcept
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return data_[i];
    }

private:
    TData data_{};
    std::mutex mutex_{};
};

//=============================================================================
//=============================================================================

template<typename TContainer>
class data_container_wrapper
{
public:
    data_container_wrapper() = default;
    auto& operator[](int i) noexcept
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return data_[i];
    }

private:
    TContainer data_{};
    std::mutex mutex_{};
};
template<typename TData>
class data_container_wrapper<std::vector<TData>>
{
public:
    data_container_wrapper() = default;
    data_container_wrapper(uint32_t p_size) { data_.reserve( p_size ); }

    auto& operator[](int i) noexcept
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return data_[i];
    }

    auto& getData() { return data_; }

private:
    std::vector<TData> data_{};
    std::mutex mutex_{};
};
//=============================================================================
//CLASSES
//=============================================================================



//=============================================================================
// TYPEDEF PARTICLES
//=============================================================================
using part = Ocacho::Physics::MassAggregate::Particle;
using cableHC = Ocacho::Physics::MassAggregate::Cable;
using rodHC = Ocacho::Physics::MassAggregate::Rod;
using gfGen = Ocacho::Physics::MassAggregate::GravityForceGenerator;
using dragGen = Ocacho::Physics::MassAggregate::DragForceGenerator;

//=============================================================================
// METHODS
//=============================================================================

void initParticles(auto& particleManager, auto& myParticles, auto& particleNodes
    , auto& smgr, auto& gravGen, const uint32_t start, const uint32_t end)
{   
    for (size_t i = start; i < end; ++i)
    {
        myParticles[i] = std::move(std::make_unique<part>());

        // Get a random number
        int random = rand() % 80 - 40;
        myParticles[i]->velocity.x = random;

        random = rand() % 80 - 40;
        myParticles[i]->velocity.y = random;

        random = rand() % 80 - 40;
        myParticles[i]->velocity.z = random;

        particleManager.addParticle(*myParticles[i].get());
        particleManager.addForceRegistration(*myParticles[i].get(), gravGen);
    }

    for (size_t i = start; i < end; ++i)
    {
        particleNodes[i] = smgr.addSphereSceneNode(1.0f);
    }
}

//=============================================================================
//=============================================================================

void initRenderNodesWSpheres(irr::scene::ISceneManager* smgr, auto& renderNodes
    , const size_t start, const size_t end)
{
    for (size_t i = start; i < end; ++i)
    {
        renderNodes[i] = smgr->addSphereSceneNode(1.0f);
    }
}

//=============================================================================
//=============================================================================

void updateNodesPosition(auto& renderNodes, auto& physEntities, const size_t start, const size_t end)
{
    for (size_t i = start; i < end; ++i)
    {
        renderNodes[i]->setPosition(irr::core::vector3df(physEntities[i]->position.x, physEntities[i]->position.y, physEntities[i]->position.z));
        
        if (physEntities[i]->position.y < -70)
        {
            physEntities[i]->velocity.y = -physEntities[i]->velocity.y;
        }
    }
}

//=============================================================================
//=============================================================================

int testExplosion()
{
  	//Cosas de Irrlicht
  	irr::IrrlichtDevice* device;
  	irr::video::IVideoDriver* driver;
  	irr::scene::ISceneManager* smgr;
 
  	device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1270, 720), 16, false, false, false, &receiver);
 
  	driver = device->getVideoDriver();
  	smgr = device->getSceneManager();
 
  	if (!device)
  		return 0;
 
  	smgr->addCameraSceneNode(0, vector3df(0,40,250), vector3df(0,0,0));
  	float x, y, z;
 
  	auto gravGen = std::make_unique<gfGen>();
  	auto drGen = std::make_unique<dragGen>();
 
  	using contactGenList = Ocacho::Meta::Typelist< cableHC, rodHC >;
  	using forceList = Ocacho::Meta::Typelist< gfGen, dragGen>;
 
  	using partMan = Ocacho::Physics::MassAggregate::ParticleManager<forceList, contactGenList>;
 
  	partMan particleManager = partMan(15, 15);

    const uint32_t particlesNumber{ 10000 };
    using particleVector_t = data_container_wrapper<std::array<std::unique_ptr<part>, particlesNumber>>;
    using particleNodes_t = data_container_wrapper<std::array<irr::scene::ISceneNode*, particlesNumber>>;

    particleVector_t myParticles {};
    particleNodes_t particleNodes {};

    srand(unsigned(time(NULL)));

    initParticles(particleManager, myParticles, particleNodes, *smgr, *gravGen.get(), 0, particlesNumber);

    /*const uint32_t numberThreads = std::thread::hardware_concurrency() - 1;
    std::vector<std::thread> myThreads;
    myThreads.reserve(numberThreads);

    const uint32_t length{ uint32_t(std::trunc( particlesNumber / numberThreads )) };
    uint32_t start{ 0 }, end{ length };

    auto lambda = [](auto&& func, auto&&... args)
        {
            func(std::forward<decltype(args)>(args)...);
        };

    for (uint32_t i = 0; i < numberThreads; ++i)
    {
        if (i == numberThreads - 1)
            end = particlesNumber;

        myThreads.emplace_back(std::thread(initParticles));

        start = end;
        end = length * (i + 2);
    }

    for (uint32_t i = 0; i < numberThreads; ++i)
    {
        myThreads[i].join();
    }*/

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    const int numberThreads = std::thread::hardware_concurrency() - 1;
    Ocacho::Physics::MassAggregate::ThreadPool myThreadPool{ numberThreads };
    //const uint32_t numberThreads = 1;
    std::vector<std::thread> myThreads;
    myThreads.reserve(numberThreads);

    const size_t length{ uint32_t(std::trunc(particlesNumber / numberThreads)) };

    Ocacho::Timer timer;
    float deltaTime;

    Ocacho::Timer renderTime;
    Ocacho::Timer threadTime;

    /*for (uint32_t i = 0; i < numberThreads; ++i)
    {
        if (i == numberThreads - 1)
            end = particlesNumber;

        myThreads.emplace_back(std::thread(updateParticlePosition<particleNodes_t, particleVector_t>, std::reference_wrapper<particleNodes_t>(particleNodes)
            , std::reference_wrapper<particleVector_t>(myParticles), start, end));

        start = end;
        end = length * (i + 2);
    }*/
    
  	while(device->run())
  	{
  		if(timer.ellapsedSeconds() >= 0.007f)
  		{
  			particleManager.reset();
 
  			deltaTime = timer.ellapsedSeconds();
  			timer.start();
 
  			particleManager.runPhysics(deltaTime);

            //myThreads.clear();

            size_t start{ 0 }, end{ length };

            threadTime.start();

            for (uint32_t i = 0; i < numberThreads; ++i)
            {
                if (i == numberThreads - 1)
                    end = particlesNumber;

                myThreadPool.doJob(std::bind(updateNodesPosition<particleNodes_t, particleVector_t>, std::reference_wrapper<particleNodes_t>(particleNodes)
                    , std::reference_wrapper<particleVector_t>(myParticles), start, end));

                start = end;
                end = length * (i + 2);
            }

            renderTime.start();
  			driver->beginScene(true, true, SColor(255,100,101,140));
 
  			smgr->drawAll();
 
  			driver->endScene();

            //printf("Render time: %fms \n", float(timer.ellapsedTime()) / 1000000);

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

            /*for (uint32_t i = 0; i < numberThreads; ++i)
            {
                myThreads[i].join();
            }*/
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
 
  	using contactGenList = Ocacho::Meta::Typelist< cableHC, rodHC >;
  	using forceList = Ocacho::Meta::Typelist< gfGen, dragGen, spring>;
 
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

//=============================================================================
// TYPEDEF RIGIDBODY
//=============================================================================
using rigidBodyT = Ocacho::Physics::RigidBody::RigidBody;
using gravityGenT = Ocacho::Physics::RigidBody::GravityForceGeneratorRB;
using forceListT = Ocacho::Meta::Typelist<gravityGenT>;
using rigidBodyManagerT = Ocacho::Physics::RigidBody::RigidBodyManager<forceListT>;

using collisionDataT = Ocacho::Physics::RigidBody::CollisionData;
using sphereColliderT = Ocacho::Physics::RigidBody::CollisionSphere;
using boxColliderT = Ocacho::Physics::RigidBody::CollisionBox;
using collisionDetectorT = Ocacho::Physics::RigidBody::CollisionDetector;
using contactResolverT = Ocacho::Physics::RigidBody::ContactResolver;

//=============================================================================
//=============================================================================

//int testRigidbody()
int main()
{
    //Cosas de Irrlicht
    irr::IrrlichtDevice* device;
    irr::video::IVideoDriver* driver;
    irr::scene::ISceneManager* smgr;

    device = createDevice(video::EDT_OPENGL, dimension2d<u32>(1270, 720), 16, false, false, false, &receiver);

    driver = device->getVideoDriver();
    smgr = device->getSceneManager();

    if (!device)
        return 0;

    smgr->addCameraSceneNode(0, vector3df(0, 20, 20), vector3df(0, 0, 0));

    const size_t rigidBodyNumber {2};
    std::array<rigidBodyT, rigidBodyNumber> rigidBodies;
    std::array<irr::scene::ISceneNode*, rigidBodyNumber> rigidBodyNodes;


    //initRenderNodesWSpheres(smgr, rigidBodyNodes, 0, rigidBodyNumber);
    rigidBodyNodes[0] = smgr->addSphereSceneNode(1.0f);
    rigidBodyNodes[1] = smgr->addCubeSceneNode(1.0f);

    gravityGenT gravityGenerator;
    rigidBodyManagerT rbManager{};
    rbManager.addRigidBody(rigidBodies[0]);
    rbManager.addRigidBody(rigidBodies[1]);
    rigidBodies[1].position.y = -20;
    rigidBodies[1].SetMass(20000000);
    rbManager.addForceRegistration(rigidBodies[0], gravityGenerator);

    collisionDetectorT colDetector;
    collisionDataT colData;
    contactResolverT contactResolver {50};
    sphereColliderT sphereCol;
    sphereCol.body_ = &rigidBodies[0];
    sphereColliderT sphereCol1;
    sphereCol1.body_ = &rigidBodies[1];
    /*boxColliderT boxCol;
    boxCol.body_ = &rigidBodies[1];
    boxCol.halfSize_ = Ocacho::Vector3(10, 1, 10);*/
    rigidBodyNodes[1]->setScale({10, 1, 10});
    rigidBodyNodes[1]->setMaterialFlag(EMF_LIGHTING, false);
    rigidBodyNodes[1]->setMaterialTexture(0, driver->getTexture("../../models/materials/red_color.png"));
    
    while (device->run())
    {
        deltaTime = timer.ellapsedSeconds();

        if (deltaTime >= 0.016f)
        {
            timer.start();

            //checkInput();
            rbManager.runPhysics(deltaTime);
            sphereCol.calculateInternals();
            sphereCol1.calculateInternals();
            colDetector.sphereAndSphere(sphereCol, sphereCol1, &colData);
            //boxCol.calculateInternals();
            //colDetector.boxAndSphere(boxCol, sphereCol, &colData);
            contactResolver.resolveContacts(colData.contactList_, colData.currentContact_, deltaTime);

            //updateNodesPosition(rigidBodyNodes, rigidBodies, 0, rigidBodyNumber);

            for (size_t i = 0; i < rigidBodyNumber; ++i)
            {
                rigidBodyNodes[i]->setPosition(irr::core::vector3df(rigidBodies[i].position.x, rigidBodies[i].position.y, rigidBodies[i].position.z));

                /*if (rigidBodies[i].position.y < -70)
                {
                    rigidBodies[i].velocity.y = -rigidBodies[i].velocity.y;
                }*/
            }

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