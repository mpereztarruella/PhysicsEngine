/**
 * @file Timer.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief Timer for Ocacho Engine.
 *
 * This class holds the implementation for the timer.
 *
 * @version 0.1
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2021
 */

#pragma once

#include "ogpch.hpp"

namespace Ocacho
{
	using clk_t = std::chrono::high_resolution_clock;
	using timePoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
	using nanos_t = std::uint64_t;

	class Timer
	{
	protected:
		//Starting time point when the timer is created
		timePoint_t	startTimePoint_;

	public:
		explicit Timer() : startTimePoint_(clk_t::now()) {};

		/**
		 * @brief Reset the startTimePoint to the current time
		 *
		 */
		void start() noexcept { startTimePoint_ = clk_t::now(); }

		/**
		 * @brief Get the ellapsed time since the last call to start()
		 *
		 * @return nanos_t Ellapsed time in nanoseconds
		 */
		nanos_t ellapsedTime() const noexcept { return(clk_t::now() - startTimePoint_).count(); }

		/**
		 * @brief Get the ellapsed time since the last call to start()
		 *
		 * @return float Ellapsed time in seconds
		 */
		float ellapsedSeconds() const noexcept { return ellapsedTime() / 1000000000.0f; }
	};
}