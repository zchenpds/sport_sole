#ifndef SPORT_SOLE_COMMON_H
#define SPORT_SOLE_COMMON_H

#include <array>


// enum left or right
enum left_right_t {
  LEFT=0,
  RIGHT,
  LEFT_RIGHT
};

// Regex to be replaced with commas: (?<![{\n )+])   (?=[ -])
namespace sport_sole {
  constexpr size_t NUM_PSENSOR = 8;
  constexpr size_t NUM_2XPSENSOR = NUM_PSENSOR * 2;
  std::array<double, NUM_2XPSENSOR> 
    Rho = {0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865, 0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865}, 
    Theta = {-0.1064, 0.0662,-0.1415, 0.0465, 0.2570,-1.1735, 1.0175, 0.2395, 0.1064,-0.0662, 0.1415,-0.0465,-0.2570, 1.1735,-1.0175,-0.2395};



  enum class GaitPhase : uint8_t {
    Swing = 0b00, // Swing
    Stance1 = 0b10, // Heel contact
    Stance2 = 0b11, // Foot flat
    Stance3 = 0b01 // Heel off
  };

  GaitPhase getGaitPhaseLR(uint8_t gait_state, left_right_t lr)
  {
    return static_cast<GaitPhase>((lr == LEFT) ? (gait_state >> 2) & 0b11 : (gait_state & 0b11));
  }

  inline bool isForefootTouchingGround(GaitPhase s)
  {
    return s == GaitPhase::Stance2 || s == GaitPhase::Stance3;
  }

  inline bool isHindfootTouchingGround(GaitPhase s)
  {
    return s == GaitPhase::Stance1 || s == GaitPhase::Stance2;
  }

  const double p_threshold = 100.0;

  // Gait Phase Finite State Machine
  template <typename structDataPacketPureDataRAW>
  struct GaitPhaseFSM
  {
    GaitPhase gait_phase;
    
    GaitPhaseFSM():
      gait_phase(GaitPhase::Stance2)
    {
    }
    
    void update(const structDataPacketPureDataRAW & data)
    {
      double p_hind_sum = data.p6 + data.p7;
      double p_fore_sum = data.p1 + data.p2 + data.p3 + data.p4 + data.p5;

      switch (gait_phase) {
        case GaitPhase::Swing:
          if (p_hind_sum > p_threshold) 
            gait_phase = GaitPhase::Stance1;
          break;
        case GaitPhase::Stance1:
          if (p_fore_sum > p_threshold) 
            gait_phase = GaitPhase::Stance2;
          break;
        case GaitPhase::Stance2:
          if (p_hind_sum <= p_threshold) 
            gait_phase = GaitPhase::Stance3;
          break;
        case GaitPhase::Stance3:
          if (p_fore_sum <= p_threshold) 
            gait_phase = GaitPhase::Swing;
          break;
      }
    }

    uint8_t getGaitPhase()
    {
      return static_cast<uint8_t>(gait_phase);
    }
  };

  struct IncidentCounter
  {
    int StickyForefootPressureSensor = 0;
    int StickyHindfootPressureSensor = 0;
  };

}

#endif // SPORT_SOLE_COMMON_H