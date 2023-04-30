#ifndef MICROTECH_SIGNALGENERATOR_HPP
#define MICROTECH_SIGNALGENERATOR_HPP

#include <cstdint>
#include "IQmathLib.h"

namespace Microtech {

#define PI 3.1415926536

/**
 * @class SignalProperties
 * @brief Class for storing properties of the signal
 */
class SignalProperties {
public:
  using PhaseType = _iq15;
  /**
   * @brief deleted default constructor
   */
  SignalProperties() = delete;
  /**
   * @brief constructor
   * @param[in] fsHz Sampling frequency of the signal
   *
   * This constructor also set the initial frequency of the signal to 1 Hz.
   */
  explicit SignalProperties(uint16_t fsHz) : samplingFreqHz(fsHz) {
    setNewFrequency(INITIAL_FREQUENCY);
  }

  /**
   * @brief set the new frequency of the signal
   * @param[in] newFrequency new frequency of the signal
   */
  void setNewFrequency(const _iq15 newFrequency) {
    currentFrequency = newFrequency;
    // 2*pi*currentFrequency/samplingFreqHz
    const _iq15 freqInCyclesPerSecond = _IQ15mpy(_IQ15(2 * PI), currentFrequency);
    phaseStep = _IQ15div(freqInCyclesPerSecond, _IQ15(samplingFreqHz));
  }

  /**
   * @brief increase the phase of the signal
   */
  void increasePhase() noexcept {
    // calculate new phase and keep it within 0 and 2pi.
    currentPhase = std::fmod((currentPhase + phaseStep), _IQ15(2 * PI));
  }

  /**
   * @brief get the current phase of the signal
   * @return current phase of the signal
   */
  PhaseType getCurrentPhase() const noexcept {
    return currentPhase;
  }

  _iq15 getCurrentFrequency() const noexcept {
    return currentFrequency;
  }

private:
  const uint16_t samplingFreqHz;  ///< Sampling frequency of the signal
  _iq15 currentFrequency = 0;     ///< Current frequency of the signal
  PhaseType phaseStep = 0;        ///< Current phase step of the signal
  PhaseType currentPhase = 0;     ///< Current phase of the signal
  static constexpr _iq15 INITIAL_FREQUENCY = _IQ15(1.0);
};

/**
 * @class iSignalType
 * @brief Interface for the different types of signals
 */
class iSignalType {
public:
  /**
   * @brief default constructor
   */
  iSignalType() = default;

  /**
   * @brief default destructor
   */
  virtual ~iSignalType() = default;

  /**
   * @brief pure virtual function to get the next point of the signal
   * @param[in] signal Signal properties
   * @return next point of the signal
   */
  virtual _iq15 getNextPoint(SignalProperties& signal) noexcept = 0;

protected:
  static constexpr _iq15 MAXIMUM_AMPLITUDE = _IQ15(100.0);
};

/**
 * @brief Class representing a Sinusoidal signal
 */
class Sinusoidal : public iSignalType {
public:
  /**
   * @brief default constructor
   */
  Sinusoidal() = default;

  /**
   * @brief get the next point of the Sinusoidal signal
   * @param[in] signal Signal properties
   * @return next point of the Sinusoidal signal
   */
  _iq15 getNextPoint(SignalProperties& signal) noexcept override {
    // sin gives value from -1 to 1, so we add 1 to get the output from 0 to 2
    const _iq15 sineValue = _IQ15sin(signal.getCurrentPhase()) + _IQ15(1);
    const _iq15 HALF_OF_MAX_AMPLITUDE = _IQ15div(MAXIMUM_AMPLITUDE, _IQ15(2.0));

    // Multiply by 50.0 since our maximum value is 100.0 and the maximum from the sine is 2.
    const _iq15 sineScaled = _IQ15mpy(sineValue, HALF_OF_MAX_AMPLITUDE);
    return sineScaled;
  }
};

/**
 * @brief Class representing Trapezoidal signal
 */
class Trapezoidal : public iSignalType {
public:
  /**
   * @brief default constructor
   */
  Trapezoidal() = default;

  /**
   * @brief get the next point of the Trapezoidal signal
   * @param[in] signal Signal properties
   * @return next point of the Trapezoidal signal
   */
  _iq15 getNextPoint(SignalProperties& signal) noexcept override {
    const _iq15 currentPhase = signal.getCurrentPhase();
    const _iq15 slope = getSlope();
    const _iq15 yIntercept = getYIntercept();

    // Use a switch case to check for the current phase
    switch (getCurrentPhase(currentPhase)) {
      case Phase::PHASE_1: return _IQ15mpy(slope, currentPhase) + yIntercept;
      case Phase::PHASE_2: return MAXIMUM_AMPLITUDE;
      case Phase::PHASE_3:
        // The last constant value has to be added to compensate since the angle is higher
        return _IQ15mpy(_IQ15(-1),_IQ15mpy(currentPhase,slope)) + yIntercept + _IQ15(300.0);
      case Phase::PHASE_4: return _IQ15(0.0);
      case Phase::PHASE_5: return _IQ15mpy(slope, currentPhase) + yIntercept - _IQ15(600.0);
      default: return _IQ15(0.0);
    }
  }

private:
  /**
   * @brief Get the slope of the Trapezoidal signal
   * @return The slope of the Trapezoidal signal
   */
  _iq15 getSlope() const noexcept {
    return _IQ15div(MAXIMUM_AMPLITUDE, DEG60_IN_RAD);
  }

  /**
   * @brief Get the y-intercept of the Trapezoidal signal
   * @return The y-intercept of the Trapezoidal signal
   */
  constexpr _iq15 getYIntercept() const noexcept {
    return _IQ15(50.0);
  }

  // define a struct to store the phase intervals
  struct IntervalPhases {
    _iq15 start;
    _iq15 end;
  };

  enum class Phase {
      PHASE_1,
      PHASE_2,
      PHASE_3,
      PHASE_4,
      PHASE_5
  };
  const IntervalPhases PHASE_1 = {DEG0_IN_RAD, DEG30_IN_RAD};
  const IntervalPhases PHASE_2 = {DEG30_IN_RAD, DEG150_IN_RAD};
  const IntervalPhases PHASE_3 = {DEG150_IN_RAD, DEG210_IN_RAD};
  const IntervalPhases PHASE_4 = {DEG210_IN_RAD, DEG330_IN_RAD};
  const IntervalPhases PHASE_5 = {DEG330_IN_RAD, DEG360_IN_RAD};

  /**
   * @brief Get the current phase of the signal
   * @param[in] currentPhase The current phase of the signal
   * @return The current phase of the signal
   */
  Phase getCurrentPhase(const _iq15 currentPhase) const {
    if (currentPhase >= PHASE_1.start && currentPhase < PHASE_1.end) {
      return Phase::PHASE_1;
    } else if (currentPhase >= PHASE_2.start && currentPhase < PHASE_2.end) {
      return Phase::PHASE_2;
    } else if (currentPhase >= PHASE_3.start && currentPhase < PHASE_3.end) {
      return Phase::PHASE_3;
    } else if (currentPhase >= PHASE_4.start && currentPhase < PHASE_4.end) {
      return Phase::PHASE_4;
    } else if (currentPhase >= PHASE_5.start && currentPhase < PHASE_5.end) {
      return Phase::PHASE_5;
    } else {
      return Phase::PHASE_1;
    }
  }

  static constexpr _iq15 DEG0_IN_RAD = _IQ15(0);
  static constexpr _iq15 DEG30_IN_RAD = _IQ15(PI / 6);
  static constexpr _iq15 DEG60_IN_RAD = _IQ15(PI / 3);
  static constexpr _iq15 DEG120_IN_RAD = _IQ15(2 * PI / 3);
  static constexpr _iq15 DEG150_IN_RAD = DEG30_IN_RAD + DEG120_IN_RAD;
  static constexpr _iq15 DEG210_IN_RAD = DEG150_IN_RAD + DEG60_IN_RAD;
  static constexpr _iq15 DEG330_IN_RAD = DEG210_IN_RAD + DEG120_IN_RAD;
  static constexpr _iq15 DEG360_IN_RAD = _IQ15(2 * PI);
};

/**
 * @brief Class representing Rectangular signal
 */
class Rectangular : public iSignalType {
public:
  /**
   * @brief default constructor
   */
  Rectangular() = default;

  /**
   * @brief get the next point of the Rectangular signal
   * @param[in] signal Signal properties
   * @return next point of the Rectangular signal
   */
  _iq15 getNextPoint(SignalProperties& signal) noexcept override {
    _iq15 retVal = 0;
    if (signal.getCurrentPhase() < _IQ15(PI)) {
      retVal = MAXIMUM_AMPLITUDE;
    } else {
      retVal = _IQ15(0);
    }
    return retVal;
  }
};

/**
 * @brief Generates different types of signals (sinusoidal, trapezoidal, and rectangular) and switch between them.
 */
class SignalGenerator {
public:
  /**
   * @enum Shape
   * @brief Enumeration of different signal shapes that can be generated.
   */
  enum class Shape {
    SINUSOIDAL,   ///< Signal shape representing sinusoidal signal
    TRAPEZOIDAL,  ///< Signal shape representing trapezoidal signal
    RECTANGULAR,  ///< Signal shape representing rectangular signal
  };

  /**
   * @brief Constructor is explicit and takes a single argument, a sampling frequency in Hz.
   * It sets the active signal to a sinusoidal signal and sets the signalProperties using the given sampling frequency.
   * @param[in] samplingFreqHz Sampling frequency in Hz
   */
  explicit SignalGenerator(uint16_t samplingFreqHz) : activeSignal(&sinusoidal), signalProperties(samplingFreqHz) {}

  /**
   * @brief get the next data point of the active signal
   * @return next data point
   */
  _iq15 getNextDatapoint() noexcept {
    signalProperties.increasePhase();
    return _IQ15mpy(activeSignal->getNextPoint(signalProperties), outputAmplitudePercentage)
           + _IQ15mpy(_IQ15(1.0) - outputAmplitudePercentage, _IQ15(100.0));
  }

  /**
   * @brief set a new frequency for the signal
   * @param[in] newFrequency new frequency
   */
  void setNewFrequency(const _iq15 newFrequency) noexcept {
    signalProperties.setNewFrequency(newFrequency);
  }

  /**
   * @brief set the active signal shape to one of the available types (sinusoidal, trapezoidal, or rectangular).
   * @param[in] newShape new shape of the signal
   */
  void setActiveSignalShape(const Shape newShape) noexcept {
    switch (newShape) {
      case Shape::SINUSOIDAL: activeSignal = &sinusoidal; break;
      case Shape::TRAPEZOIDAL: activeSignal = &trapezoidal; break;
      case Shape::RECTANGULAR: activeSignal = &rectangular; break;
    }
  }

  /**
   * @brief This function changes the active signal shape to the next shape in the list of available shapes
   */
  void nextSignalShape() {
    if (shapeIndex < 2) {
      shapeIndex++;
      setActiveSignalShape((Shape)shapeIndex);
    }
  }

  /**
   * @brief This function changes the active signal shape to the previous shape in the list of available shapes
   */
  void previousSignalShape() {
    if (shapeIndex > 0) {
      shapeIndex--;
      setActiveSignalShape((Shape)shapeIndex);
    }
  }

  /**
   * @brief This function increases the frequency of the signal by a fixed step value
   * of SignalGenerator::FREQUENCY_STEP:
   */
  void increaseFrequency() {
    const _iq15 currentFrequency = signalProperties.getCurrentFrequency();
    if (currentFrequency < MAXIMUM_FREQUENCY) {
      signalProperties.setNewFrequency(currentFrequency + FREQUENCY_STEP);
    }
  }
  /**
   * @brief This function decreases the frequency of the signal by a fixed step value
   * of SignalGenerator::FREQUENCY_STEP:
   */
  void decreaseFrequency() {
    const _iq15 currentFrequency = signalProperties.getCurrentFrequency();
    if (currentFrequency > MINIMUM_FREQUENCY) {
      signalProperties.setNewFrequency(currentFrequency - FREQUENCY_STEP);
    }
  }

  /**
   * @brief increase the amplitude of the active signal by a fixed step
   */
  void increaseAmplitude() {
    if (outputAmplitudePercentage < _IQ15(1.0)) {
      outputAmplitudePercentage += AMPLITUDE_STEP;
    }
  }

  /**
   * @brief decrease the amplitude of the active signal by a fixed step
   */
  void decreaseAmplitude() {
    if (outputAmplitudePercentage > _IQ15(0.0)) {
      outputAmplitudePercentage -= AMPLITUDE_STEP;
    }
  }

private:
  Sinusoidal sinusoidal;              ///< Sinusoidal signal object
  Trapezoidal trapezoidal;            ///< Trapezoidal signal object
  Rectangular rectangular;            ///< Rectangular signal object
  iSignalType* activeSignal;          ///< pointer to the active signal
  SignalProperties signalProperties;  ///< Signal Properties object
  _iq15 outputAmplitudePercentage =
    _IQ15(1.0);  ///< Represents the amplitude of the output signal as a percentage. It is initialized to 100%.
  uint8_t shapeIndex = 0;  ///< Represents the current shape of the active signal. It is initialized to 0 = SINUSOIDAL.

  static constexpr _iq15 MAXIMUM_FREQUENCY =
    _IQ15(5.0);  ///< Represents the maximum frequency that the signal generator can output. It is initialized to 5 Hz.
  static constexpr _iq15 MINIMUM_FREQUENCY = _IQ15(
    0.5);  ///< Represents the minimum frequency that the signal generator can output. It is initialized to 0.5 Hz.
  static constexpr _iq15 FREQUENCY_STEP =
    _IQ15(0.5);  ///< Represents the step size for frequency changes. It is initialized to 0.5 Hz.

  static constexpr _iq15 AMPLITUDE_STEP =
    _IQ15(0.05);  ///< Represents the step size for amplitude changes. It is initialized to 0.05 (5%).
};

}  // namespace Microtech

#endif  // MICROTECH_SIGNALGENERATOR_HPP
