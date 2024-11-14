/**
 * @file ParticleManager.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This class contains all the data that the manager will need to update the particles with the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <thread>
#include <queue>

#include "Utility/TypeAliases.hpp"
#include "Physics.hpp"
#include "MassAggregate/ForceGenerators/ForceGeneratorBase.hpp"
#include "MassAggregate/Contacts/ContactResolver.hpp"
#include "MassAggregate/Contacts/ContactGenerator.hpp"

namespace Ocacho::Physics::MassAggregate
{
	/**
	 * @brief Use this struct to create the two lists of parameters we'll need for the ParticleManager.
	 * The first list for the force registry that will contain all the force generator types.
	 * The second list for the contact generator vector that will contain all the contact generator types.
	 * 
	 * @tparam Ts The list of types that will contain the Typelist.
	 */
	template<typename... Ts>
	struct Typelist {};

	namespace
	{
		template <template <typename...> class New, typename List>
		struct replace {};
		template <template <typename...> class New, typename... Ts>
		struct replace<New, Typelist<Ts...>>
		{
			using type = New<Ts...>;
		};
	}

	class ThreadPool
	{
		public:

			ThreadPool(int threads) : shutdown_(false)
			{
				// Create the specified number of threads
				threads_.reserve(threads);
				for (int i = 0; i < threads; ++i)
					threads_.emplace_back(std::bind(&ThreadPool::threadEntry, this, i));
			}

			~ThreadPool()
			{
				{
					// Unblock any threads and tell them to stop
					std::unique_lock <std::mutex> l(lock_);

					shutdown_ = true;
					condVar_.notify_all();
				}

				// Wait for all threads to stop
				std::cerr << "Joining threads" << std::endl;
				for (auto& thread : threads_)
					thread.join();
			}

			void doJob(std::function <void(void)> func)
			{
				// Place a job on the queu and unblock a thread
				std::unique_lock <std::mutex> l(lock_);

				jobs_.emplace(std::move(func));
				condVar_.notify_one();
			}

		protected:

			void threadEntry(int i)
			{
				std::function <void(void)> job;

				while (1)
				{
					{
						std::unique_lock <std::mutex> l(lock_);

						while (!shutdown_ && jobs_.empty())
							condVar_.wait(l);

						if (jobs_.empty())
						{
							// No jobs to do and we are shutting down
							std::cerr << "Thread " << i << " terminates" << std::endl;
							return;
						}

						//std::cerr << "Thread " << i << " does a job" << std::endl;
						job = std::move(jobs_.front());
						jobs_.pop();
					}

					// Do the job without holding any locks
					job();
				}

			}

			std::mutex lock_;
			std::condition_variable condVar_;
			bool shutdown_;
			std::queue <std::function <void(void)>> jobs_;
			std::vector <std::thread> threads_;
	};

	template <typename ForceList, typename ContactGenList>
	class ParticleManager
	{
		private:
			using forceRegType = typename replace<ForceRegistry, ForceList>::type;
			using contactGenVariantType = typename replace<std::variant, ContactGenList>::type;

			//Holds the reference to the particles
			std::vector<Particle*> particles_;
			std::mutex particlesMutex_{};

			//Holds the force registry for the engine
			forceRegType forceReg_;
			std::mutex forceRegMutex_{};

			//Holds the contact resolver
			ContactResolver contactRes_;

			//Holds the list of contacts
			std::vector<Contact> contactList_;

			//Holds the contact generators for the manager
			std::vector<contactGenVariantType> contactGenVector_;
			std::mutex contactGenMutex_{};

			const uint32_t numberThreads_{ std::thread::hardware_concurrency() - 1 };
			//std::vector<std::thread> myThreads;
			ThreadPool threadPool_{ numberThreads_ };

		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================

			/**
			 * @brief Calls the integrator for all the particles in the particle manager.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void integrateParticles(const float p_deltaTime) noexcept;


			void integrateParticlesMultithreaded(const float p_deltaTime, const size_t p_start, const size_t p_end) noexcept;

			/**
			 * @brief Calls the addContact method for all the contact generators of the particle manager.
			 * 
			 * @return unsigned The number of contacts generated each frame.
			 */
			unsigned generateContacts() noexcept;
		
		public:
			//TODO[Otto]: De momento las iteraciones van a ser siempre dependientes de la cantidad de
			//contactos generados, en un futuro meter la opcion de que sean fijas en todos los frames.
			ParticleManager(unsigned p_maxContacts, unsigned p_iterations = 0) noexcept;

			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Call this method at start of each frame to reset the values of the manager.
			 * 
			 */
			void reset() noexcept;

			/**
			 * @brief Updates the data for the physics engine contained in the manager.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void runPhysics(const float p_deltaTime) noexcept;

			//=========================================================================
			//METHODS TO ADD DATA
			//=========================================================================

			/**
			 * @brief Call this method to add a new particle to the manager.
			 * 
			 * @param p_particle Reference to the particle that we want to add.
			 */
			void addParticle(Particle& p_particle) noexcept;

			/**
			 * @brief Call this method to add a new contact generator to the manager.
			 * 
			 * @param p_contactGen Contact generator passed by parameter, it will generate 
			 * 	a copy so it can be deleted in other sites.
			 */
			void addContactGenerator(auto p_contactGen) noexcept;

			/**
			 * @brief Call this method to add a new Force Registration that links a force generator
			 * with a particle.
			 * 
			 * @param p_particle Reference to the particle to which we want to link with a force generator.
			 * @param p_forceGen Reference to the force generator that we want to link the particle with.
			 */
			void addForceRegistration(Particle& p_particle, auto& p_forceGen) noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "ParticleManager.tpp"
