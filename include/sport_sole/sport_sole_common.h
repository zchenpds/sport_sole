#ifndef SPORT_SOLE_COMMON_H
#define SPORT_SOLE_COMMON_H

#include <array>

// Regex to be replaced with commas: (?<![{\n )+])   (?=[ -])
namespace sport_sole {
  constexpr size_t NUM_PSENSOR = 8;
  constexpr size_t NUM_2XPSENSOR = NUM_PSENSOR * 2;
  std::array<double, NUM_2XPSENSOR> 
    Rho = {0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865, 0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865}, 
    Theta = {-0.1064, 0.0662,-0.1415, 0.0465, 0.2570,-1.1735, 1.0175, 0.2395, 0.1064,-0.0662, 0.1415,-0.0465,-0.2570, 1.1735,-1.0175,-0.2395};
}

#endif // SPORT_SOLE_COMMON_H