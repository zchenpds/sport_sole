#ifndef SPORT_SOLE_COMMON_H
#define SPORT_SOLE_COMMON_H

#include <array>
#include <sport_sole/SportSole.h>


// enum left or right
enum left_right_t {
  LEFT=0,
  RIGHT,
  LEFT_RIGHT
};

enum fore_hind_t {
  FORE=0,
  HIND=1,
  FORE_HIND
};

// Regex to be replaced with commas: (?<![{\n )+])   (?=[ -])
namespace sport_sole {
  constexpr size_t NUM_PSENSOR = 8;
  constexpr size_t NUM_2XPSENSOR = NUM_PSENSOR * 2;
  std::array<double, NUM_2XPSENSOR> 
    Rho =    {0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0865, 0.0174, 0.0128, 0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0865, 0.0174, 0.0128}, 
    Theta = {-0.1064, 0.0662,-0.1415, 0.0465, 0.2570, 0.2395,-1.1735, 1.0175, 0.1064,-0.0662, 0.1415,-0.0465,-0.2570,-0.2395, 1.1735,-1.0175};



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

  constexpr double P_THRESHOLD = 100.0;
  double p_threshold = P_THRESHOLD;

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
      double p_hind_sum = data.p7 + data.p8;
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

  class GaitPhaseFSM2 {
    GaitPhase gait_phases_[LEFT_RIGHT];
    double p_threshold_;
    
  public:
    GaitPhaseFSM2(double p_threshold = P_THRESHOLD): 
      gait_phases_{GaitPhase::Stance2, GaitPhase::Stance2},
      p_threshold_(p_threshold)
    {}
      
    void update(const sport_sole::SportSole& msg)
    {
      const auto& pressures = msg.pressures;
      for (size_t lr : {LEFT, RIGHT}) {
        size_t i0 = (lr==LEFT ? 0 : 8);
        double p_hind_sum = pressures[i0 + 6] + pressures[i0 + 7]; // 7~8
        double p_fore_sum = pressures[i0 + 0] + pressures[i0 + 1] + pressures[i0 + 2] + pressures[i0 + 3] + pressures[i0 + 4]; // 1~5
        double wy = msg.angular_velocity[lr].y;

        auto & gait_phase = gait_phases_[lr];
        switch (gait_phase) {
          case GaitPhase::Swing:
            if (p_hind_sum > p_threshold_)
              gait_phase = GaitPhase::Stance1;
            break;
          case GaitPhase::Stance1:
            if (p_fore_sum > p_threshold_)
              gait_phase = GaitPhase::Stance2;
            break;
          case GaitPhase::Stance2:
            if (p_hind_sum <= p_threshold_ && fabs(msg.angular_velocity[lr].y) > 1.0)
              gait_phase = GaitPhase::Stance3;
            break;
          case GaitPhase::Stance3:
            if (p_fore_sum <= p_threshold_) 
              gait_phase = GaitPhase::Swing;
            break;
        }
      }
    }

    uint8_t getGaitState() const
    {
      return static_cast<uint8_t>(gait_phases_[LEFT]) << 2 | static_cast<uint8_t>(gait_phases_[RIGHT]);
    }
  };


  struct IncidentCounter
  {
    int StickyForefootPressureSensor = 0;
    int StickyHindfootPressureSensor = 0;
  };

}

#endif // SPORT_SOLE_COMMON_H