/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file M5UnitUnifiedTHERMO.hpp
  @brief Main header of M5UnitTHERMO using M5UnitUnified

  @mainpage M5Unit-THERMO
  Library for UnitTHERMO using M5UnitUnified.
*/
#ifndef M5_UNIT_UNIFIED_THERMO_HPP
#define M5_UNIT_UNIFIED_THERMO_HPP

#include "unit/unit_MLX90614.hpp"
#include "unit/unit_NCIR2.hpp"
#include "unit/unit_Thermal2.hpp"

/*!
  @namespace m5
  @brief Top level namespace of M5stack
 */
namespace m5 {

/*!
  @namespace unit
  @brief Unit-related namespace
 */
namespace unit {

using UnitNCIR = m5::unit::UnitMLX90614BAA;

}  // namespace unit
}  // namespace m5
#endif
