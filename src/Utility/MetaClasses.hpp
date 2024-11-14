/**
 * @file ParticleManager.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This file contains all the classes needed for setting up the engine
 *
 *
 * @version 0.1
 * @date 2022-01-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

namespace Ocacho::Meta
{
	template<typename... Ts>
	struct Typelist {};

	template <template <typename...> class New, typename List>
	struct replace {};
	template <template <typename...> class New, typename... Ts>
	struct replace<New, Typelist<Ts...>>
	{
		using type = New<Ts...>;
	};
}